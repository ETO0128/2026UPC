"""Select replayed P2 schedules and build the trajectory figure and budget."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results/p2-p0-v1"


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def select(strategies, name):
    policy = strategies.get(name, {})
    accepted = []
    if policy.get("result", {}).get("passes"):
        accepted.append((policy["result"], "feedback policy search", "policy-search.json", policy["convergence"]))
    path = RESULTS / f"{name}-collocation.json"
    diagnostics = []
    if path.exists():
        for index, row in enumerate(load(path)["history"]):
            if "candidate" not in row:
                diagnostics.append(row)
                continue
            solver = row["candidate"]
            coarse, fine = row["replay"]["coarse"], row["replay"]["fine"]
            diagnostics.append(dict(index=index, method=solver.get("method", "Hermite-Simpson collocation"),
                                    gridTargetS=solver["maxIntervalTargetS"], status=solver["status"],
                                    propellantKg=solver["propellantKg"], verified=coarse["passes"] and fine["passes"],
                                    failedChecks=[k for k, failed in fine["violations"].items() if failed]))
            if solver["success"] and coarse["passes"] and fine["passes"]:
                accepted.append((fine, diagnostics[-1]["method"], f"{path.name} history[{index}]", dict(
                    coarserStepS=coarse["stepS"], coarserTimeS=coarse["timeS"],
                    coarserFinalState=coarse["finalState"], coarserConsumedKg=coarse["consumedKg"])))
    if not accepted:
        return dict(verified=False, policyTrials=policy.get("trials", 0), optimizationTrials=diagnostics), None
    result, method, source, convergence = min(accepted, key=lambda x: x[0]["propellantKg"])
    summary = {key: value for key, value in result.items() if key != "trace"}
    summary.update(verified=True, method=method, source=source, stepComparison=convergence,
                   policyTrials=policy["trials"], optimizationTrials=diagnostics)
    return summary, result["trace"]


def plot(result, trace):
    time = np.array([row["t"] for row in trace])
    state = np.array([row["state"] for row in trace])
    throttle = np.array([row["throttle"] for row in trace])
    q = np.array([row["qPa"] for row in trace])
    load_g = np.array([row["loadG"] for row in trace])
    fig, axes = plt.subplots(2, 3, figsize=(11, 6), constrained_layout=True)
    path, velocity, attitude, control, pressure, mass = axes.ravel()
    path.plot(state[:, 0] / 1000, state[:, 1] / 1000, color="#126b76")
    path.set(xlabel="Downrange (km)", ylabel="COM altitude (km)")
    for phase in result["phases"]:
        if phase["powered"]:
            path.scatter(phase["startState"][0] / 1000, phase["startState"][1] / 1000, color="#aa501d", s=23)
            for ax in (velocity, attitude, control, pressure, mass):
                ax.axvspan(phase["startS"], phase["endS"], alpha=0.12, color="#e99a59")
    velocity.plot(time, state[:, 2], label="$v_x$")
    velocity.plot(time, state[:, 3], label="$v_h$")
    velocity.set(ylabel="Velocity (m/s)")
    velocity.legend(frameon=False)
    attitude.plot(time, np.rad2deg(state[:, 5]), color="#126b76")
    attitude.axhline(90, ls="--", color="0.5", lw=0.8)
    attitude.set(ylabel="Body pitch (deg)")
    control.plot(time, throttle, color="#126b76")
    control.set(ylabel="Throttle", ylim=(-0.04, 1.08))
    control_twin = control.twinx()
    control_twin.plot(time, np.rad2deg([row["gimbalRad"] for row in trace]), lw=0.8, color="#aa501d")
    control_twin.set_ylabel("Gimbal (deg)", color="#aa501d")
    pressure.plot(time, q / 1000, color="#126b76")
    pressure.set(ylabel="Dynamic pressure (kPa)")
    pressure_twin = pressure.twinx()
    pressure_twin.plot(time, load_g, lw=0.8, color="#aa501d")
    pressure_twin.set_ylabel("Proper acceleration (g)", color="#aa501d")
    dry_mass = result["initialState"][4] - result["propellantKg"]
    mass.plot(time, (state[:, 4] - dry_mass) / 1000, color="#126b76")
    mass.set(ylabel="Propellant remaining (t)")
    for ax in (velocity, attitude, control, pressure, mass):
        ax.set_xlabel("Time from separation (s)")
    for ax in axes.ravel():
        ax.grid(alpha=0.23)
    fig.savefig(ROOT / "figure/p2-reference.png", dpi=185)
    plt.close(fig)


def main():
    strategies = load(RESULTS / "policy-search.json")
    result = dict(contractVersion=strategies["contractVersion"], strategies={})
    traces = {}
    for name in ("single", "double", "level", "steep"):
        result["strategies"][name], traces[name] = select(strategies, name)
    steep = strategies["steepCoastScreen"]
    result["steepCoastScreen"] = {key: steep[key] for key in ("event", "timeS", "finalState")}
    names = [name for name in ("single", "double") if result["strategies"][name]["verified"]]
    if not names:
        raise SystemExit("Neither nominal strategy passed P1 replay")
    best = min(names, key=lambda name: result["strategies"][name]["propellantKg"])
    result["nominalSelected"] = best
    plot(result["strategies"][best], traces[best])
    (RESULTS / "summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
