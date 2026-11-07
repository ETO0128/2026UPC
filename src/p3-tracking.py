"""Prepare a lower-demand reference and feedback gains for fixed-ship tracking."""

import importlib.util
import json
from pathlib import Path
import subprocess

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
STATE_WEIGHTS = [.001, 1, 1, 10, .001, 1, 1]
TERMINAL_WEIGHTS = [.1, 1000, 100, 1000, .001, 10000, 10000]
CONTROL_WEIGHTS = [10, 10]
spec = importlib.util.spec_from_file_location("p2_collocate", ROOT / "src/p2-collocate.py")
p2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p2)


def node(mode, request):
    result = subprocess.run(["node", "src/p3-tracking.mjs", mode], cwd=ROOT,
                            input=json.dumps(request), encoding="utf-8", stdout=subprocess.PIPE, check=True)
    return json.loads(result.stdout)


def prepare_reference(source, params, contract):
    ignition_time = source["replay"]["phases"][0]["endS"] - 1
    request = dict(mode="observe", change={}, propellantKg=source["propellantKg"],
                   shipLocationM=source["candidate"]["shipLocationM"], coastDurationS=ignition_time)
    observed = subprocess.run(["node", "src/p3-replan.mjs"], cwd=ROOT, input=json.dumps(request),
                              encoding="utf-8", stdout=subprocess.PIPE, check=True)
    coast = json.loads(observed.stdout)
    burn = source["candidate"]["phases"][-1]
    duration = burn["knots"][-1] - burn["knots"][0]
    error = np.array(coast["finalState"]) - np.array(burn["states"][0])
    trace = []
    for t, state, throttle, gimbal in zip(burn["knots"], burn["states"], burn["throttle"], burn["gimbalRad"]):
        elapsed = t - burn["knots"][0]
        trace.append(dict(t=elapsed, phase=burn["name"], throttle=throttle, gimbalRad=gimbal,
                          state=(np.array(state) + (1 - elapsed / duration) * error).tolist()))
    seed = dict(initialState=coast["finalState"], finalState=trace[-1]["state"], trace=trace,
                phases=[dict(name=burn["name"], powered=True, startS=0, endS=duration)])
    candidate = p2.optimize(params, contract, seed, .3, 450, shooting=True,
                            fixed_initial_mass=True, fixed_ship=source["candidate"]["shipLocationM"],
                            throttle_ceiling=.95, gimbal_fraction=.9, control_smoothing=.001,
                            touchdown_vertical_speed=-1)
    for phase in candidate["phases"]:
        phase["knots"] = [t + ignition_time for t in phase["knots"]]
    candidate["phases"].insert(0, dict(name="coast", powered=False, knots=[0, ignition_time],
                                      states=[coast["initialState"], coast["finalState"]],
                                      throttle=[0, 0], gimbalRad=[0, 0]))
    return candidate


def feedback_gains(model):
    transitions = np.array(model["a"]).transpose(0, 2, 1)
    controls = np.array(model["b"]).transpose(0, 2, 1)
    q = np.diag(STATE_WEIGHTS) * model["dt"]
    r = np.diag(CONTROL_WEIGHTS) * model["dt"]
    value = np.diag(TERMINAL_WEIGHTS)
    gains = []
    for a, b in zip(reversed(transitions), reversed(controls)):
        gain = np.linalg.solve(r + b.T @ value @ b, b.T @ value @ a)
        value = q + a.T @ value @ (a - b @ gain)
        value = (value + value.T) / 2
        gains.append(gain.tolist())
    return gains[::-1]


def main():
    directory = ROOT / "results/p3/tracking"
    directory.mkdir(exist_ok=True)
    reference_path = directory / "reference.json"
    if not reference_path.exists():
        source = p2.load(ROOT / "results/p3/replanned/nominal.json")
        params = p2.load(ROOT / "config/reference.json")
        contract = p2.load(ROOT / "config/p0-contract.json")
        candidate = prepare_reference(source, params, contract)
        if not candidate["success"]:
            raise RuntimeError(f"Reference optimization did not converge: {candidate['status']}")
        reference = dict(candidate=candidate, coastTimeS=0,
                         propellantKg=source["propellantKg"], shipLocationM=source["candidate"]["shipLocationM"],
                         throttleCeiling=.95, gimbalFraction=.9, controlSmoothing=.001, targetVerticalMps=-1,
                         ignitionAdvanceS=1)
        node("check", reference)
        reference_path.write_text(json.dumps(reference, indent=2) + "\n", encoding="utf-8")
    reference = p2.load(reference_path)
    model = node("linearize", reference)
    controller = dict(dt=model["dt"], states=model["states"], derivatives=model["derivatives"],
                      gains=feedback_gains(model), stateScale=model["stateScale"],
                      weights=dict(state=STATE_WEIGHTS, terminal=TERMINAL_WEIGHTS, control=CONTROL_WEIGHTS))
    (directory / "controller.json").write_text(json.dumps(controller, indent=2) + "\n", encoding="utf-8")
    coarse = node("simulate", dict(reference=reference, controller=controller, stepS=.005))
    output = node("simulate", dict(reference=reference, controller=controller, stepS=.0025))
    output["coarserStepS"] = .005
    output["coarserCases"] = coarse["cases"]
    (directory / "summary.json").write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    for case in output["cases"]:
        print(case["actualWindMps"], {name: result["passes"] for name, result in case["results"].items()}, flush=True)
    if not all(case["results"]["feedback"]["passes"] for result in (coarse, output) for case in result["cases"]):
        raise RuntimeError("At least one tracking comparison failed its touchdown criteria")


if __name__ == "__main__":
    main()
