"""Reoptimize separation inventory for the three P0 effective-Isp scenarios."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / "results/p3/propulsion"
spec = importlib.util.spec_from_file_location("p2_collocate", ROOT / "src/p2-collocate.py")
p2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p2)


def replay(candidate, isp, step):
    request = dict(mode="execute", change={"ispS": isp}, propellantKg=candidate["propellantKg"],
                   shipLocationM=candidate["shipLocationM"], phases=candidate["phases"], stepS=step)
    result = subprocess.run(["node", "src/p3-replan.mjs"], cwd=ROOT, input=json.dumps(request),
                            encoding="utf-8", capture_output=True, check=True)
    return json.loads(result.stdout)


def accepted(row):
    return row["candidate"]["success"] and all(item["passes"] for item in row["replay"].values())


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def summarize():
    params = p2.load(ROOT / "config/reference.json")
    contract = p2.load(ROOT / "config/p0-contract.json")
    cases = []
    for path in sorted(DIRECTORY.glob("isp-*.json")):
        source = p2.load(path)
        passing = [row for row in source["history"] if accepted(row)]
        if not passing:
            cases.append(dict(ispS=source["ispS"], accepted=False,
                              attempts=len(source["history"]), source=path.relative_to(ROOT).as_posix()))
            continue
        best = min(passing, key=lambda row: row["candidate"]["propellantKg"])
        candidate = best["candidate"]
        result = best["replay"]["0.0025"]
        ignition = next(phase for phase in result["phases"] if phase["powered"])
        retained = result["touchdown"]["reserveKg"]
        cases.append(dict(ispS=source["ispS"], accepted=True, attempts=len(source["history"]),
                          source=path.relative_to(ROOT).as_posix(), selectedHistoryIndex=source["history"].index(best),
                          propellantKg=candidate["propellantKg"], consumedKg=result["consumedKg"],
                          retainedUsableKg=retained, methaneKg=result["methaneKg"], liquidOxygenKg=result["liquidOxygenKg"],
                          shipLocationM=candidate["shipLocationM"], ignitionTimeS=ignition["startS"],
                          ignitionAltitudeM=ignition["startState"][1], contactTimeS=result["timeS"],
                          burnDurationS=result["timeS"] - ignition["startS"],
                          touchdown=result["touchdown"], peakQPa=result["peakQPa"], peakLoadG=result["peakLoadG"],
                          solverStatus=candidate["status"], stepTargetS=candidate["maxIntervalTargetS"]))
    baseline = next((case for case in cases if case["ispS"] == params["ispS"] and case["accepted"]), None)
    if baseline:
        for case in cases:
            if case["accepted"]:
                case["loadDifferenceFromReferenceKg"] = case["propellantKg"] - baseline["propellantKg"]
                case["loadChangePercent"] = 100 * case["loadDifferenceFromReferenceKg"] / baseline["propellantKg"]
                case["fixedReferenceImpulseInventoryKg"] = (baseline["retainedUsableKg"]
                    + baseline["consumedKg"] * baseline["ispS"] / case["ispS"])
    output = dict(parameter="ispS", scenarioValues=contract["sensitivity"]["ispS"],
                  comparison="Known effective Isp; recovery site selected separately for each design, fixed during replay",
                  minimumUsableReserveKg=contract["touchdown"]["usableReserveMinKg"],
                  oxygenFuelRatio=params["oxygenFuelRatio"], maximumThrustN=params["maxThrustN"],
                  interpretation="Lowest accepted local candidate per scenario; no structural or thermal limits imposed",
                  replayStepsS=[.005, .0025], cases=cases)
    write_json(DIRECTORY / "summary.json", output)
    for case in cases:
        print("SUMMARY", case["ispS"], case["accepted"], case.get("propellantKg"), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isp", type=int, choices=[300, 330, 360])
    parser.add_argument("--step", type=float, default=.55)
    parser.add_argument("--max-iter", type=int, default=400)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--warm-start", type=Path)
    parser.add_argument("--summarize", action="store_true")
    args = parser.parse_args()
    if args.step <= 0 or args.max_iter < 1:
        parser.error("Use a positive mesh interval and iteration limit")
    DIRECTORY.mkdir(exist_ok=True, parents=True)
    if args.summarize:
        summarize()
        return
    if args.isp is None:
        parser.error("Select one scenario with --isp, or use --summarize")
    params = p2.load(ROOT / "config/reference.json")
    params["ispS"] = args.isp
    contract = p2.load(ROOT / "config/p0-contract.json")
    original = p2.load(ROOT / "results/p2-p0-v1/single-collocation.json")["history"][4]
    path = DIRECTORY / f"isp-{args.isp}.json"
    history = p2.load(path)["history"] if path.exists() else []
    candidate = original["candidate"]
    origin = "results/p2-p0-v1/single-collocation.json history[4]"
    if args.resume and history:
        converged = [row for row in history if row["candidate"]["success"]]
        candidate = (min(converged, key=lambda row: row["candidate"]["propellantKg"])
                     if converged else history[-1])["candidate"]
        origin = f"{path.relative_to(ROOT).as_posix()} prior converged candidate"
    elif args.warm_start:
        source = p2.load(args.warm_start)
        candidate = source["candidate"]
        origin = str(args.warm_start)
    started = time.perf_counter()
    reuse_reference = args.isp == 330 and not args.resume and not args.warm_start
    if not reuse_reference:
        seed = p2.from_candidate(candidate, {})
        print(f"START Isp={args.isp} s, grid={args.step} s, seed={origin}", flush=True)
        candidate = p2.optimize(params, contract, seed, args.step, args.max_iter, shooting=True)
    checked = {str(step): replay(candidate, args.isp, step) for step in (.005, .0025)}
    row = dict(candidate=candidate, replay=checked, elapsedSeconds=time.perf_counter() - started,
               seedSource=origin, reusedP2Reference=reuse_reference)
    history.append(row)
    write_json(path, dict(ispS=args.isp, casadi=p2.ca.__version__, history=history))
    print(f"DONE Isp={args.isp}: {candidate['status']}, load={candidate['propellantKg']:.3f} kg, "
          f"accepted={accepted(row)}, seconds={row['elapsedSeconds']:.1f}", flush=True)
    for step, result in checked.items():
        print(step, result["event"], result["violations"], result["touchdown"], flush=True)
    summarize()


if __name__ == "__main__":
    main()
