"""Optimize P0-v1 separation inventory, then replay schedules in P1 Node.js."""

import argparse
import json
from pathlib import Path
import subprocess

import casadi as ca
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCALE = np.array([300_000, 80_000, 1_500, 1_000, 150_000, np.pi, 1])
R = 6_356_766.0
G0 = 9.80665
R_AIR = 287.05287
LAYERS = [0, 11_000, 20_000, 32_000, 47_000, 51_000, 71_000, 84_852]
LAPSES = [-0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002]


def load(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def p1_rhs(p):
    """Symbolic transcription of planar.mjs and atmosphere.mjs for derivatives."""
    state = ca.SX.sym("y", 7)
    u = ca.SX.sym("u", 2)
    _, h, vx, vh, mass, theta, omega = ca.vertsplit(state)
    geopotential = R * h / (R + h)
    temperature, pressure = 288.15, 101325.0
    rho = 0
    for i, lapse in enumerate(LAPSES):
        delta = geopotential - LAYERS[i]
        temp = temperature + lapse * delta
        if lapse:
            # Keep unused branches differentiable outside their own layers.
            pres = pressure * (ca.fmax(temp, 100) / temperature) ** (-G0 / (R_AIR * lapse))
        else:
            pres = pressure * ca.exp(-G0 * delta / (R_AIR * temperature))
        layer_rho = pres / (R_AIR * ca.fmax(temp, 100))
        rho = layer_rho if i == 0 else ca.if_else(geopotential >= LAYERS[i], layer_rho, rho)
        width = LAYERS[i + 1] - LAYERS[i]
        upper = temperature + lapse * width
        pressure *= ((upper / temperature) ** (-G0 / (R_AIR * lapse)) if lapse
                     else np.exp(-G0 * width / (R_AIR * temperature)))
        temperature = upper

    length, diameter = p["lengthM"], p["diameterM"]
    cos_theta, sin_theta = ca.cos(theta), ca.sin(theta)
    axial = (vx - p["windMps"]) * cos_theta + vh * sin_theta
    normal = -(vx - p["windMps"]) * sin_theta + vh * cos_theta
    axial_force = -0.5 * rho * p["axialCd"] * np.pi * diameter**2 / 4 * axial * ca.fabs(axial)
    strip_width = length / p["strips"]
    arm = ca.DM(-length / 2 + (np.arange(p["strips"]) + 0.5) * strip_width + p["cpOffsetM"])
    local_velocity = normal + omega * arm
    strip_force = -0.5 * rho * p["crossflowCd"] * diameter * strip_width * local_velocity * ca.fabs(local_velocity)
    normal_force = ca.sum1(strip_force)
    thrust = u[0] * p["maxThrustN"]
    delta = u[1] * np.deg2rad(p["gimbalLimitDeg"])
    dry_inertia = p["dryMassKg"] * (3 * (diameter / 2)**2 + length**2) / 12
    return ca.Function("p1_scaled_rhs", [state, u], [ca.vertcat(
        vx, vh,
        (axial_force * cos_theta - normal_force * sin_theta + thrust * ca.cos(theta + delta)) / mass,
        -G0 * (R / (R + h))**2
        + (axial_force * sin_theta + normal_force * cos_theta + thrust * ca.sin(theta + delta)) / mass,
        -thrust / (G0 * p["ispS"]), omega,
        (ca.dot(arm, strip_force) - length / 2 * thrust * ca.sin(delta)) / dry_inertia,
    ) / ca.DM(SCALE)])


def seed_knots(seed, phase, fraction, gimbal_limit):
    start = phase["startS"]
    end = phase["endS"]
    time = start + (end - start) * fraction
    rows = [row for row in seed["trace"] if row["phase"] == phase["name"]
            and start - 1e-8 <= row["t"] <= end + 1e-8]
    source = np.array([row["t"] for row in rows])
    states = np.array([row["state"] for row in rows])
    controls = np.array([[row["throttle"], row["gimbalRad"] / gimbal_limit] for row in rows])
    return (np.array([np.interp(time, source, v) for v in states.T]) / SCALE[:, None],
            np.array([np.interp(time, source, v) for v in controls.T]))


def rk_interval(rhs, substeps):
    """RK4 transcription of a piecewise-linear control interval."""
    state = ca.MX.sym("state", 7)
    u0 = ca.MX.sym("u0", 2)
    u1 = ca.MX.sym("u1", 2)
    duration = ca.MX.sym("duration")
    y = state
    h = duration / substeps
    for i in range(substeps):
        u = u0 + (u1 - u0) * (i / substeps)
        middle = u0 + (u1 - u0) * ((i + 0.5) / substeps)
        end = u0 + (u1 - u0) * ((i + 1) / substeps)
        k1 = rhs(ca.DM(SCALE) * y, u)
        k2 = rhs(ca.DM(SCALE) * (y + h * k1 / 2), middle)
        k3 = rhs(ca.DM(SCALE) * (y + h * k2 / 2), middle)
        k4 = rhs(ca.DM(SCALE) * (y + h * k3), end)
        y = y + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return ca.Function("rk4_interval", [state, u0, u1, duration], [y])


def optimize(p, contract, seed, step, max_iter, *, shooting=False, altitude_buffer=0, substeps=8,
             fixed_initial_mass=False, fixed_ship=None, throttle_ceiling=1, gimbal_fraction=1,
             control_smoothing=0, touchdown_vertical_speed=-1.5):
    if not p["minThrottle"] <= throttle_ceiling <= 1 or not 0 < gimbal_fraction <= 1 or control_smoothing < 0:
        raise ValueError("Planning limits must fit inside the reference actuator limits")
    nlp = ca.Opti()
    rhs = p1_rhs(p)
    step_function = rk_interval(rhs, substeps) if shooting else None
    scale = ca.repmat(ca.DM(SCALE), 1, 1)
    gimbal_limit = np.deg2rad(p["gimbalLimitDeg"])
    floor = p["dryMassKg"] + contract["budget"]["unusableResidualKg"] + contract["touchdown"]["usableReserveMinKg"]
    altitude_limit = R * LAYERS[-1] / (R - LAYERS[-1]) - altitude_buffer
    stages = []

    def bounds(states, clearance=False):
        nlp.subject_to(nlp.bounded(p["deckHeightM"] / SCALE[1], states[1, :], altitude_limit / SCALE[1]))
        nlp.subject_to(states[4, :] >= floor / SCALE[4])
        if clearance:
            theta = states[5, :] * SCALE[5]
            extent = p["lengthM"] / 2 * ca.fabs(ca.sin(theta)) + p["diameterM"] / 2 * ca.fabs(ca.cos(theta))
            nlp.subject_to(states[1, :] >= (p["deckHeightM"] + extent) / SCALE[1])

    for phase in seed["phases"]:
        guess_duration = phase["endS"] - phase["startS"]
        powered = phase["powered"]
        intervals = max(5, int(np.ceil(guess_duration / (step if powered else step * 1.4))))
        fraction = np.linspace(0, 1, intervals + 1)
        if not powered:
            fraction = 1 - (1 - fraction)**1.6
        duration = nlp.variable()
        lower = max(0.001, guess_duration * 0.85) if shooting else 0.001
        upper = min(400, guess_duration * 1.15) if shooting else 400
        nlp.subject_to(nlp.bounded(lower / 100, duration, upper / 100))
        nlp.set_initial(duration, guess_duration / 100)
        x = nlp.variable(7, intervals + 1)
        x_guess, u_guess = seed_knots(seed, phase, fraction, gimbal_limit)
        nlp.set_initial(x, x_guess)
        if stages:
            nlp.subject_to(x[:, 0] == stages[-1][2][:, -1])
        else:
            fixed = np.arange(7) if fixed_initial_mass else np.r_[0:4, 5:7]
            nlp.subject_to(x[fixed, 0] == np.array(seed["initialState"])[fixed] / SCALE[fixed])
        if powered:
            u = nlp.variable(2, intervals + 1)
            nlp.subject_to(nlp.bounded(p["minThrottle"], u[0, :], throttle_ceiling))
            nlp.subject_to(nlp.bounded(-gimbal_fraction, u[1, :], gimbal_fraction))
            nlp.set_initial(u, u_guess)
        else:
            u = ca.DM.zeros(2, intervals + 1)
        physical_x = ca.repmat(scale, 1, intervals + 1) * x
        f = rhs.map(intervals + 1)(physical_x, u)
        durations = 100 * duration * ca.DM(np.diff(fraction)).T
        dt = ca.repmat(durations, 7, 1)
        mid = (x[:, :-1] + x[:, 1:]) / 2 + dt / 8 * (f[:, :-1] - f[:, 1:])
        if shooting:
            integrated = step_function.map(intervals)(x[:, :-1], u[:, :-1], u[:, 1:], durations)
            nlp.subject_to(x[:, 1:] == integrated)
            nlp.subject_to(nlp.bounded(-0.15, x[5, :] - ca.DM(x_guess[5:6, :]), 0.15))
        else:
            mid_f = rhs.map(intervals)(ca.repmat(scale, 1, intervals) * mid, (u[:, :-1] + u[:, 1:]) / 2)
            nlp.subject_to(x[:, 1:] - x[:, :-1] == dt / 6 * (f[:, :-1] + 4 * mid_f + f[:, 1:]))
        bounds(x, clearance=phase is not seed["phases"][-1])
        if phase is seed["phases"][-1]:
            bounds(x[:, :-1], clearance=True)
        bounds(mid, clearance=True)
        stages.append((phase, duration, x, u, fraction))

    first, final = stages[0][2][:, 0], stages[-1][2][:, -1]
    # Interior targets leave room for numerical differences on replay.
    nlp.subject_to(final[1:4] == np.array([p["deckHeightM"] + p["lengthM"] / 2, 0, touchdown_vertical_speed]) / SCALE[1:4])
    upright = 0.5 + 2 * round((seed["finalState"][5] - np.pi / 2) / (2 * np.pi))
    nlp.subject_to(final[5:7] == [upright, 0])
    nlp.subject_to(final[4] >= (floor + 75) / SCALE[4])
    if fixed_ship is not None:
        nlp.subject_to(final[0] == fixed_ship / SCALE[0])
    objective = first[4] - final[4] if fixed_initial_mass else first[4]
    if control_smoothing:
        objective += control_smoothing * sum(ca.sumsqr(stage[3][:, 1:] - stage[3][:, :-1]) for stage in stages if stage[0]["powered"])
    nlp.minimize(objective)
    nlp.solver("ipopt", {"print_time": False}, {"print_level": 0, "max_iter": max_iter,
               "max_wall_time": 140, "tol": 1e-8, "acceptable_tol": 2e-6,
               "acceptable_iter": 10, "bound_relax_factor": 0})

    solution = nlp.solve_limited()
    info = solution.stats()
    phases = []
    elapsed = 0.0
    for phase, duration, x, u, fraction in stages:
        time = 100 * float(solution.value(duration))
        controls = np.asarray(solution.value(u))
        states = (np.asarray(solution.value(x)) * SCALE[:, None]).T
        phases.append(dict(name=phase["name"], powered=phase["powered"], knots=(elapsed + time * fraction).tolist(),
                           states=states.tolist(), throttle=controls[0].tolist(),
                           gimbalRad=(controls[1] * gimbal_limit).tolist()))
        elapsed += time
    return dict(status=info["return_status"], success=bool(info["success"]), iterations=info["iter_count"],
                method="RK4 multiple shooting" if shooting else "Hermite-Simpson collocation",
                atmosphereBufferM=altitude_buffer, rkSubsteps=substeps if shooting else None,
                maxIntervalTargetS=step, propellantKg=float(solution.value(first[4]) * SCALE[4] - p["dryMassKg"]),
                shipLocationM=phases[-1]["states"][-1][0], phases=phases)


def run_replay(candidate, angle):
    request = dict(candidate, flightPathDeg=angle, samples=True)
    process = subprocess.run(["node", "src/p2-replay.mjs"], cwd=ROOT, input=json.dumps(request),
                             encoding="utf-8", capture_output=True, check=True)
    return json.loads(process.stdout)


def from_candidate(candidate, result):
    """Resample the prior collocation solution onto the next mesh."""
    seed = dict(result, phases=[], trace=[])
    seed["initialState"] = candidate["phases"][0]["states"][0]
    seed["finalState"] = candidate["phases"][-1]["states"][-1]
    for phase in candidate["phases"]:
        seed["phases"].append(dict(name=phase["name"], powered=phase["powered"],
                                   startS=phase["knots"][0], endS=phase["knots"][-1]))
        seed["trace"].extend(dict(t=t, phase=phase["name"], state=state, throttle=throttle, gimbalRad=gimbal)
                             for t, state, throttle, gimbal in zip(phase["knots"], phase["states"],
                                                                 phase["throttle"], phase["gimbalRad"]))
    return seed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("strategy", choices=["single", "double", "level", "steep"])
    parser.add_argument("--steps", nargs="+", type=float, default=[1.5, 0.6, 0.25])
    parser.add_argument("--max-iter", type=int, default=450)
    parser.add_argument("--shooting-step", type=float)
    parser.add_argument("--atmosphere-buffer-m", type=float, default=0)
    parser.add_argument("--rk-substeps", type=int, default=8)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "results/p2-p0-v1")
    args = parser.parse_args()
    if (any(step <= 0 for step in args.steps) or (args.shooting_step is not None and args.shooting_step <= 0)
            or args.atmosphere_buffer_m < 0 or args.rk_substeps < 1):
        parser.error("Mesh intervals and substep count must be positive and the atmosphere buffer nonnegative")
    contract = load(ROOT / "config/p0-contract.json")
    p = load(ROOT / contract["parameterFile"])
    policy = load(args.output / "policy-search.json")[args.strategy]
    if "result" not in policy:
        raise SystemExit(f"No policy seed for {args.strategy}")
    seed = policy["result"]
    angle = {"level": 0, "steep": 30}.get(args.strategy, p["flightPathDeg"])
    path = args.output / f"{args.strategy}-collocation.json"
    history = load(path)["history"] if args.resume else []
    if args.resume:
        args.steps = []

    def save():
        for row in history:
            if "replay" in row:
                row["replay"]["coarse"]["trace"] = []
                if not row["replay"]["fine"]["passes"]:
                    row["replay"]["fine"]["trace"] = []
        path.write_text(json.dumps(dict(contract=contract["version"], casadi=ca.__version__, history=history)) + "\n",
                        encoding="utf-8")

    def check(solution, step):
        checked = run_replay(solution, angle)
        fine = checked["fine"]
        print(f"{args.strategy} {solution['method']} {step}: {solution['status']}, {solution['propellantKg']:.3f} kg, "
              f"P1 replay {fine['event']}, {fine['violations']}, final {fine['finalState']}", flush=True)
        # Only the finer replay supplies trajectory samples for the report.
        checked["coarse"]["trace"] = []
        if not fine["passes"]:
            fine["trace"] = []
        history.append(dict(candidate=solution, replay=checked))
        save()
        return fine

    for step in args.steps:
        try:
            solution = optimize(p, contract, seed, step, args.max_iter, altitude_buffer=args.atmosphere_buffer_m)
        except RuntimeError as error:
            print(f"{args.strategy} grid {step}: {error}", flush=True)
            history.append(dict(maxIntervalTargetS=step, error=str(error)))
            continue
        fine = check(solution, step)
        if solution["success"]:
            seed = from_candidate(solution, fine)
    if args.shooting_step:
        candidates = [row for row in history if row.get("candidate", {}).get("success")]
        if candidates:
            # Start from the best converged collocation whose replay gets near the deck.
            best = min(candidates, key=lambda row: row["candidate"]["propellantKg"]
                       + 500 * max(0, row["replay"]["fine"]["finalState"][1] - 40)
                       + 100 * abs(row["replay"]["fine"]["finalState"][3] + 1.5))
            seed = from_candidate(best["candidate"], best["replay"]["fine"])
        solution = optimize(p, contract, seed, args.shooting_step, args.max_iter, shooting=True,
                            altitude_buffer=args.atmosphere_buffer_m, substeps=args.rk_substeps)
        check(solution, args.shooting_step)


if __name__ == "__main__":
    main()
