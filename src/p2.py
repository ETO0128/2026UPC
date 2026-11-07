"""Find, optimize, and independently replay provisional A/P2 strategies."""

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import sys

import numpy as np
import scipy
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares, minimize

from p2_model import G0, Scenario, dynamics, flight_limits, forces


ROOT = Path(__file__).resolve().parents[1]
STATE_SCALE = np.array([100_000, 70_000, 1_000, 1_000, 130_000])
TIME_SCALE = 60.0
COAST = np.zeros((2, 2))
PRESSURE_BACKOFF = 0.005
LOAD_BACKOFF = 0.005


@dataclass
class Phase:
    name: str
    duration: float
    controls: np.ndarray

    def control_at(self, fraction):
        grid = np.linspace(0, 1, len(self.controls))
        return np.stack([np.interp(fraction, grid, column) for column in self.controls.T], axis=-1)


def integrate(scenario, initial_mass, phases, *, fine=False):
    """Forward solve with a separate integrator step at each control knot."""
    state = scenario.initial_state(initial_mass)
    trajectory = []
    for phase in phases:
        times = np.linspace(0, phase.duration, len(phase.controls))

        def rhs(t, state):
            return dynamics(scenario, state, phase.control_at(t / phase.duration))

        options = dict(method="DOP853", rtol=2e-10, atol=1e-8, max_step=0.15) if fine else dict(
            rtol=2e-8, atol=1e-7, max_step=3.0,
        )
        if fine:
            # Restart at control knots so a slope change cannot fall inside a step.
            dense_segments = []
            for a, b in zip(times[:-1], times[1:]):
                solution = solve_ivp(rhs, (a, b), state, dense_output=True, **options)
                if not solution.success:
                    raise RuntimeError(solution.message)
                dense_segments.append(solution)
                state = solution.y[:, -1]
            trajectory.append(dense_segments)
        else:
            solution = solve_ivp(rhs, (0, phase.duration), state, dense_output=True, **options)
            if not solution.success:
                raise RuntimeError(solution.message)
            trajectory.append(solution)
            state = solution.y[:, -1]
    return trajectory


def terminal_error(scenario, state):
    return state[1:4] - [scenario.touchdown_altitude_m, 0, scenario.touchdown_vertical_speed_m_s]


def reference(scenario):
    """Find a single-burn seed by solving for four terminal quantities."""
    max_tilt = np.deg2rad(scenario.max_thrust_tilt_deg)
    throttle_min = scenario.min_thrust_n / scenario.max_thrust_n

    def schedule(z):
        mass, coast, burn, throttle, initial_tilt, middle_tilt = z
        thrust = scenario.max_thrust_n * throttle
        return mass * STATE_SCALE[4], [
            Phase("coast", coast * TIME_SCALE, COAST),
            Phase("landing", burn * TIME_SCALE,
                  np.array([[thrust, initial_tilt], [thrust, middle_tilt], [thrust, 0]])),
        ]

    def residual(z):
        mass, phases = schedule(z)
        estimated_final_mass = mass - phases[-1].duration * phases[-1].controls[0, 0] / (
            G0 * scenario.specific_impulse_s
        )
        if estimated_final_mass <= 0.5 * scenario.dry_mass_kg:
            # Keep integrator trials away from the singularity at zero mass.
            return np.full(16, 1 + (0.5 * scenario.dry_mass_kg - estimated_final_mass) / 10_000)
        trajectory = integrate(scenario, mass, phases)
        final = trajectory[-1].y[:, -1]
        errors = list(terminal_error(scenario, final) / [300, 100, 100])
        # Seed above the reserve floor to allow room for later optimization.
        errors.append((final[4] - scenario.minimum_mass - 50) / 10_000)
        for phase, solution in zip(phases, trajectory):
            fraction = np.linspace(0, 1, 91)
            state = solution.sol(phase.duration * fraction).T
            limits = flight_limits(scenario, state, phase.control_at(fraction))
            limits[:, -2:] -= [PRESSURE_BACKOFF, LOAD_BACKOFF]
            errors.extend(4 * np.minimum(limits.min(axis=0), 0))
            rate = np.max(np.abs(np.diff(phase.controls[:, 1]))) * (len(phase.controls) - 1) / phase.duration
            errors.append(max(0, rate / np.deg2rad(scenario.max_powered_tilt_rate_deg_s) - 1))
        return np.array(errors)

    bounds = (
        [scenario.minimum_mass / STATE_SCALE[4], 0.05, 0.05, throttle_min, -max_tilt, -max_tilt],
        [4.0, 3.0, 3.0, 1.0, max_tilt, max_tilt],
    )
    best = None
    for coast, throttle in ((45, 0.58), (55, 0.62), (35, 0.65)):
        guess = [1.8, coast / TIME_SCALE, 0.75, max(throttle, throttle_min), -1.0, -0.8]
        guess = np.clip(guess, *bounds)
        fit = least_squares(residual, guess, bounds=bounds, diff_step=1e-4, max_nfev=170,
                            ftol=1e-8, xtol=1e-8, gtol=1e-8)
        mass, phases = schedule(fit.x)
        result, _ = replay(scenario, mass, phases)
        if result["passes_replay"]:
            return mass, phases
        if best is None or fit.cost < best[0]:
            best = fit.cost, mass, phases
    # A nonfeasible reference can still initialize the two-burn search.
    return best[1:]


class Collocation:
    """Multiphase Hermite-Simpson defects with linear thrust and angle."""

    def __init__(self, scenario, phases, intervals):
        self.scenario = scenario
        self.names = [phase.name for phase in phases]
        self.intervals = intervals
        self.starts = np.r_[0, np.cumsum(intervals)]
        self.state_size = (self.starts[-1] + 1) * len(STATE_SCALE)
        self.control_scale = np.array([scenario.max_thrust_n, 1])

    def variables(self, z):
        count = len(self.names)
        batch = z.shape[:-1]
        durations = z[..., :count] * TIME_SCALE
        states = z[..., count:count + self.state_size].reshape(*batch, -1, 5) * STATE_SCALE
        commands = []
        cursor = count + self.state_size
        for name, n in zip(self.names, self.intervals):
            if name == "coast":
                controls = np.zeros((*batch, n + 1, 2))
            else:
                end = cursor + 2 * (n + 1)
                controls = z[..., cursor:end].reshape(*batch, n + 1, 2) * self.control_scale
                cursor = end
            commands.append(controls)
        return states, durations, commands

    def unpack(self, z):
        states, durations, commands = self.variables(z)
        phases = [Phase(name, duration, COAST if name == "coast" else controls)
                  for name, duration, controls in zip(self.names, durations, commands)]
        return states, phases

    def seed(self, mass, phases):
        integrated = integrate(self.scenario, mass, phases)
        states = []
        controls = []
        for i, (phase, solution, n) in enumerate(zip(phases, integrated, self.intervals)):
            fractions = np.linspace(0, 1, n + 1)
            states.extend(solution.sol(phase.duration * fractions[i > 0:]).T / STATE_SCALE)
            if phase.name != "coast":
                controls.extend(phase.control_at(fractions) / self.control_scale)
        durations = [phase.duration / TIME_SCALE for phase in phases]
        return np.concatenate((durations, np.ravel(states), np.ravel(controls)))

    def bounds(self):
        s = self.scenario
        bounds = [(1.0 / TIME_SCALE, 180.0 / TIME_SCALE)] * len(self.names)
        states = [(-4, 4), (s.touchdown_altitude_m / STATE_SCALE[1], 86_000 / STATE_SCALE[1]),
                  (-3, 3), (-3, 3), (s.minimum_mass / STATE_SCALE[4], 4)]
        bounds.extend(states * (self.starts[-1] + 1))
        tilt = np.deg2rad(s.max_thrust_tilt_deg)
        touchdown_tilt = np.deg2rad(s.touchdown_thrust_tilt_deg)
        throttle = (s.min_thrust_n / s.max_thrust_n, 1)
        for name, n in zip(self.names, self.intervals):
            if name != "coast":
                bounds.extend([throttle, (-tilt, tilt)] * (n + 1))
                if name == "landing":
                    bounds[-1] = (-touchdown_tilt, touchdown_tilt)
        return np.array(bounds)

    def constraints(self, z):
        s = self.scenario
        batch = z.shape[:-1]
        states, durations, commands = self.variables(z)
        initial = s.initial_state(0)
        final_target = [s.touchdown_altitude_m, 0, s.touchdown_vertical_speed_m_s]
        equality = [(states[..., 0, :4] - initial[:4]) / STATE_SCALE[:4],
                    (states[..., -1, 1:4] - final_target) / STATE_SCALE[1:4]]
        inequality = []
        for i, (name, controls, start, end) in enumerate(
                zip(self.names, commands, self.starts[:-1], self.starts[1:])):
            nodes = states[..., start:end + 1, :]
            dt = durations[..., i, None, None] / (end - start)
            f = dynamics(s, nodes, controls)
            mids = 0.5 * (nodes[..., :-1, :] + nodes[..., 1:, :]) + dt / 8 * (f[..., :-1, :] - f[..., 1:, :])
            mid_controls = 0.5 * (controls[..., :-1, :] + controls[..., 1:, :])
            f_mid = dynamics(s, mids, mid_controls)
            defect = np.diff(nodes, axis=-2) - dt / 6 * (f[..., :-1, :] + 4 * f_mid + f[..., 1:, :])
            equality.append((defect / STATE_SCALE).reshape(*batch, -1))
            for sample, command in ((nodes, controls), (mids, mid_controls)):
                margins = flight_limits(s, sample, command)
                margins[..., -2:] -= [PRESSURE_BACKOFF, LOAD_BACKOFF]
                # Node altitude and mass are already covered by box bounds.
                if sample is nodes:
                    margins = margins[..., -2:]
                inequality.append(margins.reshape(*batch, -1))
            if name != "coast":
                change = np.diff(controls[..., 1], axis=-1)
                rate_limit = np.deg2rad(s.max_powered_tilt_rate_deg_s) * dt[..., 0]
                inequality.extend((rate_limit - change, rate_limit + change))
        return np.concatenate(equality, axis=-1), np.concatenate(inequality, axis=-1)

    def solve(self, mass, phases):
        bounds = self.bounds()
        guess = np.clip(self.seed(mass, phases), *bounds.T)
        index = len(self.names) + 4
        gradient = np.zeros_like(guess)
        gradient[index] = 1
        identity = np.eye(len(guess))
        step = np.sqrt(np.finfo(float).eps)
        cached_z = None
        cached_jacobians = None

        def jacobian(z, kind):
            # Evaluate all finite differences in a batch, shared by both constraints.
            nonlocal cached_z, cached_jacobians
            if cached_z is None or not np.array_equal(z, cached_z):
                before = self.constraints(z)
                direction = np.where(z + step > bounds[:, 1], -step, step)
                perturbed = z + direction * identity
                actual_steps = (perturbed.diagonal() - z)[:, None]
                after = self.constraints(perturbed)
                cached_jacobians = tuple(((a - b) / actual_steps).T for a, b in zip(after, before))
                cached_z = z.copy()
            return cached_jacobians[kind]

        result = minimize(lambda z: z[index], guess, jac=lambda z: gradient,
                          method="SLSQP", bounds=bounds, constraints=[
                              {"type": "eq", "fun": lambda z: self.constraints(z)[0],
                               "jac": lambda z: jacobian(z, 0)},
                              {"type": "ineq", "fun": lambda z: self.constraints(z)[1],
                               "jac": lambda z: jacobian(z, 1)},
                          ], options={"ftol": 2e-9, "maxiter": 220})
        equality, inequality = self.constraints(result.x)
        defect = float(np.max(np.abs(equality)))
        violation = float(max(0, -np.min(inequality)))
        states, phases = self.unpack(result.x)
        metadata = dict(success=bool(result.success), iterations=int(result.nit), message=str(result.message),
                        scaled_equality_error=defect, scaled_inequality_violation=violation,
                        intervals=list(self.intervals))
        return float(states[0, 4]), phases, metadata


def replay(scenario, mass, phases):
    """Return dense forward checks and a trace including both sides of ignition."""
    trajectory = integrate(scenario, mass, phases, fine=True)
    rows = []
    phase_summary = []
    elapsed = 0.0
    for phase, segments in zip(phases, trajectory):
        for segment_index, segment in enumerate(segments):
            start, end = segment.t[0], segment.t[-1]
            time = np.linspace(start, end, max(3, int(np.ceil((end - start) / 0.05)) + 1))
            if segment_index:
                time = time[1:]
            states = segment.sol(time).T
            control = phase.control_at(time / phase.duration)
            pressure, force = forces(scenario, states, control)
            load = np.linalg.norm(force, axis=-1) / (states[:, 4] * G0)
            rows.extend(zip(time + elapsed, [phase.name] * len(time), *states.T,
                            *control.T, pressure, load))
        used = segments[0].y[4, 0] - segments[-1].y[4, -1]
        phase_summary.append(dict(phase=phase.name, start_s=elapsed, duration_s=phase.duration,
                                  consumed_kg=float(used)))
        elapsed += phase.duration
    final = trajectory[-1][-1].y[:, -1]
    error = terminal_error(scenario, final)
    array = np.asarray([row[:1] + row[2:] for row in rows], dtype=float)
    q_peak, load_peak = np.max(array[:, -2:], axis=0)
    mass_min = np.min(array[:, 5])
    min_altitude = np.min(array[:, 2])
    max_altitude = np.max(array[:, 2])
    thrust_ok = all(
        np.all(phase.controls[:, 0] == 0) if phase.name == "coast" else
        (np.min(phase.controls[:, 0]) >= scenario.min_thrust_n - 1e-5
         and np.max(phase.controls[:, 0]) <= scenario.max_thrust_n + 1e-5
         and np.max(np.abs(phase.controls[:, 1])) <= np.deg2rad(scenario.max_thrust_tilt_deg) + 1e-8
         and np.max(np.abs(np.diff(phase.controls[:, 1]))) * (len(phase.controls) - 1) / phase.duration
         <= np.deg2rad(scenario.max_powered_tilt_rate_deg_s) + 1e-8)
        for phase in phases
    )
    touchdown_tilt = abs(np.rad2deg(phases[-1].controls[-1, 1]))
    expected_use = sum(phase.duration * np.trapezoid(phase.controls[:, 0], dx=1 / (len(phase.controls) - 1))
                       / (G0 * scenario.specific_impulse_s) for phase in phases)
    mass_error = mass - final[4] - expected_use
    # Small tolerances describe numerical endpoint agreement with the target.
    passes = bool(np.all(np.abs(error) <= [0.30, 0.10, 0.10])
                  and q_peak <= scenario.dynamic_pressure_limit_pa
                  and load_peak <= scenario.proper_acceleration_limit_g
                  and mass_min >= scenario.minimum_mass - 0.05
                  and min_altitude >= scenario.touchdown_altitude_m - 0.30
                  and max_altitude <= 86_000 and thrust_ok
                  and touchdown_tilt <= scenario.touchdown_thrust_tilt_deg + 1e-8
                  and abs(mass_error) <= 0.05)
    propellant = mass - scenario.dry_mass_kg
    methane = propellant / (1 + scenario.oxidizer_to_fuel_ratio)
    summary = dict(
        passes_replay=passes, separation_mass_kg=mass, separation_propellant_kg=propellant,
        separation_methane_kg=methane, separation_lox_kg=propellant - methane,
        consumed_kg=float(mass - final[4]), remaining_propellant_kg=float(final[4] - scenario.dry_mass_kg),
        planned_reserve_kg=scenario.reserve_kg, unusable_kg=scenario.unusable_kg,
        extra_remaining_kg=float(final[4] - scenario.minimum_mass), mass_balance_error_kg=float(mass_error),
        touchdown_x_m=float(final[0]), touchdown_altitude_m=float(final[1]),
        touchdown_vx_m_s=float(final[2]), touchdown_vz_m_s=float(final[3]),
        touchdown_thrust_tilt_deg=float(touchdown_tilt), terminal_residual=error.tolist(),
        peak_dynamic_pressure_pa=float(q_peak), peak_proper_acceleration_g=float(load_peak),
        min_altitude_m=float(min_altitude), max_altitude_m=float(max_altitude),
        phases=phase_summary,
    )
    return summary, rows


def with_entry_burn(scenario, phases, fraction):
    coast, landing = phases
    first = max(1.0, fraction * coast.duration)
    entry_time = 2.0
    controls = np.array([[scenario.min_thrust_n, -0.9], [scenario.min_thrust_n, -0.9]])
    return [Phase("coast", first, COAST), Phase("entry", entry_time, controls),
            Phase("coast", max(1.0, coast.duration - first - entry_time), COAST), landing]


def optimize_strategy(scenario, label, starts, grids):
    history = []

    def run(mass, phases, grid, initial_guess):
        mass, phases, solver = Collocation(scenario, phases, grid).solve(mass, phases)
        summary, rows = replay(scenario, mass, phases)
        history.append(dict(initial_guess=initial_guess, solver=solver,
                            separation_propellant_kg=summary["separation_propellant_kg"],
                            terminal_residual=summary["terminal_residual"],
                            peak_dynamic_pressure_pa=summary["peak_dynamic_pressure_pa"],
                            peak_proper_acceleration_g=summary["peak_proper_acceleration_g"],
                            passes_replay=summary["passes_replay"]))
        print(f"{label} {grid} {initial_guess}: {summary['separation_propellant_kg'] / 1000:.3f} t, "
              f"replay h error {summary['terminal_residual'][0]:+.3f} m, "
              f"q {summary['peak_dynamic_pressure_pa'] / 1000:.2f} kPa, "
              f"solver={solver['success']}, replay={summary['passes_replay']}", flush=True)
        return mass, phases, solver, summary, rows

    def converged(solver):
        return (solver["success"] and solver["scaled_equality_error"] < 1e-6
                and solver["scaled_inequality_violation"] < 1e-6)

    candidates = [run(mass, phases, grids[0], name) for name, mass, phases in starts]
    valid = [candidate for candidate in candidates if converged(candidate[2])]
    if valid:
        current = min(valid, key=lambda candidate: candidate[0])
    else:
        current = min(candidates, key=lambda candidate: candidate[2]["scaled_equality_error"]
                      + candidate[2]["scaled_inequality_violation"])
    verified = [candidate for candidate in valid if candidate[3]["passes_replay"]]
    for grid in grids[1:]:
        last_mass, phases = current[:2]
        current = run(last_mass, phases, grid, "previous grid")
        if converged(current[2]) and current[3]["passes_replay"]:
            verified.append(current)
            if abs(current[0] - last_mass) < 150:
                break
    mass, phases, solver, summary, rows = verified[-1] if verified else current
    summary["solver"] = solver
    summary["solver_history"] = history
    return mass, phases, summary, rows


def export_strategy(output, label, phases, summary, rows):
    trace_path = output / f"{label}_trajectory.csv"
    controls_path = output / f"{label}_controls.csv"
    summary["trajectory_file"] = trace_path.name
    summary["control_file"] = controls_path.name
    with trace_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "phase", "x_m", "altitude_m", "vx_m_s", "vz_m_s", "mass_kg",
                         "thrust_n", "tilt_rad", "dynamic_pressure_pa", "proper_acceleration_g"])
        writer.writerows(([f"{value:.9g}" if not isinstance(value, str) else value for value in row])
                         for row in rows)
    with controls_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["phase_index", "phase", "time_s", "thrust_n", "tilt_rad"])
        elapsed = 0.0
        for index, phase in enumerate(phases):
            for time, (thrust, tilt) in zip(np.linspace(0, phase.duration, len(phase.controls)), phase.controls):
                writer.writerow([index, phase.name, f"{elapsed + time:.14g}", f"{thrust:.14g}", f"{tilt:.14g}"])
            elapsed += phase.duration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, default=ROOT / "src" / "p2_scenario.json")
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "p2")
    parser.add_argument("--reference-only", action="store_true")
    args = parser.parse_args()
    parameters = json.loads(args.scenario.read_text(encoding="utf-8"))
    scenario = Scenario(**parameters)
    mass, phases = reference(scenario)
    reference_summary, reference_rows = replay(scenario, mass, phases)
    print(f"Reference: {reference_summary['separation_propellant_kg'] / 1000:.3f} t, "
          f"replay={reference_summary['passes_replay']}", flush=True)
    if args.reference_only:
        if not reference_summary["passes_replay"]:
            raise SystemExit("No single-burn reference passed replay for this scenario.")
        return

    # A distinct longer-coast, shorter-burn guess is resampled by the same dynamics.
    delay = min(12, 0.25 * phases[1].duration)
    delayed = [Phase("coast", phases[0].duration + delay, COAST),
               Phase("landing", phases[1].duration - delay, phases[1].controls)]
    single_mass, single_phases, single_summary, single_rows = optimize_strategy(
        scenario, "single_burn", [("reference", mass, phases), ("delayed ignition", mass, delayed)],
        [(12, 12), (20, 24), (32, 40), (48, 60)],
    )
    _, double_phases, double_summary, double_rows = optimize_strategy(
        scenario, "entry_and_landing", [
            ("early entry", single_mass, with_entry_burn(scenario, single_phases, 0.25)),
            ("later entry", single_mass, with_entry_burn(scenario, single_phases, 0.65)),
        ], [(5, 5, 8, 12), (8, 8, 14, 24), (12, 12, 22, 40), (18, 18, 32, 60)],
    )
    args.output.mkdir(parents=True, exist_ok=True)
    summaries = {}
    for label, commands, summary, rows in (
        ("reference", phases, reference_summary, reference_rows),
        ("single_burn", single_phases, single_summary, single_rows),
        ("entry_and_landing", double_phases, double_summary, double_rows),
    ):
        export_strategy(args.output, label, commands, summary, rows)
        summaries[label] = summary
    document = dict(parameters=parameters, python=sys.version.split()[0], numpy=np.__version__,
                    scipy=scipy.__version__, strategies=summaries)
    (args.output / "summary.json").write_text(json.dumps(document, indent=2, allow_nan=False) + "\n",
                                              encoding="utf-8")
    if any(not summary["passes_replay"] or not summary.get("solver", {}).get("success", True)
           for summary in summaries.values()):
        raise SystemExit("Diagnostics exported. At least one strategy failed convergence or forward replay.")


if __name__ == "__main__":
    main()
