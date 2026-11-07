"""Plan a fixed-ship correction at separation using a predicted descent waypoint."""

import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("p2_collocate", ROOT / "src/p2-collocate.py")
p2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p2)


def replay(request):
    result = subprocess.run(["node", "src/p3-replan.mjs"], cwd=ROOT, input=json.dumps(request),
                            encoding="utf-8", stdout=subprocess.PIPE, check=True)
    return json.loads(result.stdout)


def remaining_seed(original, observed, coast):
    """Use an actual unpowered coast and match the P2 burn to its endpoint."""
    burn = original["candidate"]["phases"][1]
    reference_start = np.array(burn["states"][0])
    ignition_state = np.array(coast["finalState"])
    # Preserve the observed revolution count while matching equivalent body orientations.
    pitch_shift = 2 * np.pi * round((ignition_state[5] - reference_start[5]) / (2 * np.pi))
    reference_start[5] += pitch_shift
    error = ignition_state - reference_start
    duration = burn["knots"][-1] - burn["knots"][0]
    trace = list(coast["trace"])
    for time, values, throttle, gimbal in zip(burn["knots"], burn["states"], burn["throttle"], burn["gimbalRad"]):
        elapsed = time - burn["knots"][0]
        state = np.array(values)
        state[5] += pitch_shift
        state += (1 - elapsed / duration) * error
        trace.append(dict(t=coast["timeS"] + elapsed, phase=burn["name"], state=state.tolist(),
                          throttle=throttle, gimbalRad=gimbal))
    return dict(initialState=observed, finalState=trace[-1]["state"], trace=trace,
                phases=[dict(name="coast", powered=False, startS=0, endS=coast["timeS"]),
                        dict(name=burn["name"], powered=True, startS=coast["timeS"], endS=coast["timeS"] + duration)])


def replay_all():
    directory = ROOT / "results/p3/replanned"
    ship = p2.load(ROOT / "results/p2-p0-v1/summary.json")["strategies"]["single"]["shipLocationM"]
    original = p2.load(ROOT / "results/p2-p0-v1/single-collocation.json")["history"][4]["candidate"]
    cases = []
    for path in sorted(directory.glob("*.json")):
        if path.name == "summary.json":
            continue
        case = p2.load(path)
        prediction = case["coastPrediction"]
        base = dict(mode="execute", change=case["change"], propellantKg=case["propellantKg"],
                    shipLocationM=ship, coastTimeS=prediction["timeS"], phases=case["candidate"]["phases"])
        checked = {str(step): replay(dict(base, stepS=step)) for step in (.005, .0025)}
        unadjusted = replay(dict(mode="execute", change=case["change"], propellantKg=case["propellantKg"],
                                 shipLocationM=ship, phases=original["phases"]))
        time_to_ignition = prediction["timeS"] + case["candidate"]["phases"][0]["knots"][-1]
        cases.append(dict(case=case["case"], propellantKg=case["propellantKg"],
                          solverSucceeded=case["candidate"]["success"],
                          timeToIgnitionS=time_to_ignition, replay=checked,
                          unadjustedAtSameInventory=unadjusted))
        print(case["case"], [(step, result["passes"]) for step, result in checked.items()], flush=True)
    output = dict(planning="At separation from the measured initial state and known uniform wind",
                  shipLocationM=ship, cases=cases)
    (directory / "summary.json").write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--propellant-kg", type=float, default=30000)
    parser.add_argument("--prediction-altitude-m", type=float, default=25000)
    parser.add_argument("--replay-all", action="store_true")
    parser.add_argument("--case", choices=["nominal", "wind-minus", "wind-plus", "pitch-minus", "pitch-plus",
                                         "wind-large-minus", "wind-large-plus", "pitch-large-minus", "pitch-large-plus",
                                         "rate-minus", "rate-plus"], default="nominal")
    args = parser.parse_args()
    if args.replay_all:
        replay_all()
        return
    params = p2.load(ROOT / "config/reference.json")
    contract = p2.load(ROOT / "config/p0-contract.json")
    summary = p2.load(ROOT / "results/p2-p0-v1/summary.json")["strategies"]["single"]
    original = p2.load(ROOT / "results/p2-p0-v1/single-collocation.json")["history"][4]
    ignition_altitude = original["candidate"]["phases"][1]["states"][0][1]
    if args.propellant_kg <= 0 or not ignition_altitude < args.prediction_altitude_m < params["altitudeM"]:
        parser.error("Use a positive inventory and a prediction altitude between nominal ignition and separation")
    changes = {"nominal": {}, "wind-minus": {"windMps": -1}, "wind-plus": {"windMps": 1},
               "pitch-minus": {"pitchDeg": params["pitchDeg"] - .1}, "pitch-plus": {"pitchDeg": params["pitchDeg"] + .1},
               "wind-large-minus": {"windMps": -10}, "wind-large-plus": {"windMps": 10},
               "pitch-large-minus": {"pitchDeg": params["pitchDeg"] - 2}, "pitch-large-plus": {"pitchDeg": params["pitchDeg"] + 2},
               "rate-minus": {"pitchRateRadS": -float(np.deg2rad(1))}, "rate-plus": {"pitchRateRadS": float(np.deg2rad(1))}}
    change = changes[args.case]
    params.update(change)
    base = dict(change=change, propellantKg=args.propellant_kg, shipLocationM=summary["shipLocationM"])
    started = time.perf_counter()
    prediction = replay(dict(base, mode="observe", observationAltitudeM=args.prediction_altitude_m))
    if abs(prediction["finalState"][1] - args.prediction_altitude_m) > .01:
        raise RuntimeError("The predicted coast did not reach the planning waypoint")
    coast = replay(dict(base, mode="observe", startState=prediction["finalState"],
                        observationAltitudeM=ignition_altitude, samples=True))
    seed = remaining_seed(original, prediction["finalState"], coast)
    candidate = p2.optimize(params, contract, seed, .4, 350, shooting=True,
                            fixed_initial_mass=True, fixed_ship=summary["shipLocationM"])
    solve_seconds = time.perf_counter() - started
    checked = replay(dict(base, mode="execute", coastTimeS=prediction["timeS"], phases=candidate["phases"]))
    directory = ROOT / "results/p3/replanned"
    directory.mkdir(exist_ok=True)
    time_to_ignition = prediction["timeS"] + candidate["phases"][0]["knots"][-1]
    output = dict(case=args.case, propellantKg=args.propellant_kg, change=change,
                  planningStartsAtSeparation=True, predictionAltitudeM=args.prediction_altitude_m, coastPrediction=prediction,
                  candidate=candidate, planningSeconds=solve_seconds,
                  timeToIgnitionS=time_to_ignition,
                  readyBeforeIgnition=solve_seconds < time_to_ignition, replay=checked)
    (directory / f"{args.case}.json").write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(args.case, candidate["status"], checked["event"], checked["passes"], checked["touchdown"], flush=True)


if __name__ == "__main__":
    main()
