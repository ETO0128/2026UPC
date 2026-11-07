"""Bounded closeout of two P0 crossflow-drag scenarios, without changing P2."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / "results/p3/drag"
spec = importlib.util.spec_from_file_location("p2_collocate", ROOT / "src/p2-collocate.py")
p2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p2)


def replay(candidate, cd, step):
    request = dict(mode="execute", change={"crossflowCd": cd}, propellantKg=candidate["propellantKg"],
                   shipLocationM=candidate["shipLocationM"], phases=candidate["phases"], stepS=step)
    result = subprocess.run(["node", "src/p3-replan.mjs"], cwd=ROOT, input=json.dumps(request),
                            encoding="utf-8", capture_output=True, check=True)
    return json.loads(result.stdout)


def accepted(row):
    return bool(row.get("candidate", {}).get("success")) and bool(row.get("replay")) and all(
        result["passes"] for result in row["replay"].values())


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def summarize():
    params = p2.load(ROOT / "config/reference.json")
    cases = []
    for path in sorted(DIRECTORY.glob("cd-*.json")):
        source = p2.load(path)
        good = [(i, row) for i, row in enumerate(source["history"]) if accepted(row)]
        selected = min(good, key=lambda item: item[1]["candidate"]["propellantKg"]) if good else None
        item = dict(crossflowCd=source["crossflowCd"], accepted=selected is not None,
                    attempts=len(source["history"]), source=path.relative_to(ROOT).as_posix())
        if selected:
            index, row = selected
            candidate, result = row["candidate"], row["replay"]["0.0025"]
            ignition = next(phase for phase in result["phases"] if phase["powered"])
            item.update(selectedHistoryIndex=index, propellantKg=candidate["propellantKg"],
                        consumedKg=result["consumedKg"], retainedUsableKg=result["touchdown"]["reserveKg"],
                        methaneKg=result["methaneKg"], liquidOxygenKg=result["liquidOxygenKg"],
                        shipLocationM=candidate["shipLocationM"], ignitionTimeS=ignition["startS"],
                        ignitionAltitudeM=ignition["startState"][1], contactTimeS=result["timeS"],
                        touchdown=result["touchdown"], peakQPa=result["peakQPa"], peakLoadG=result["peakLoadG"],
                        solverStatus=candidate["status"])
        else:
            last = source["history"][-1]
            item.update(solverStatus=last.get("candidate", {}).get("status", "solver error"),
                        diagnosticReplay=last.get("replay"), error=last.get("error"))
        cases.append(item)
    write_json(DIRECTORY / "summary.json", dict(parameter="crossflowCd", scenarioValues=[.8, 1.2, 1.6],
        comparison="Known coefficient, separately selected recovery site for each nominal design",
        axialCd=params["axialCd"], ispS=params["ispS"], minimumUsableReserveKg=5000,
        replayStepsS=[.005, .0025], interpretation="Accepted local candidates only; unsuccessful search is not physical infeasibility",
        cases=cases))
    for case in cases:
        print("SUMMARY", case["crossflowCd"], case["accepted"], case.get("propellantKg"), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cd", type=float, choices=[.8, 1.2, 1.6])
    parser.add_argument("--summarize", action="store_true")
    args = parser.parse_args()
    DIRECTORY.mkdir(parents=True, exist_ok=True)
    if args.summarize:
        summarize()
        return
    if args.cd is None:
        parser.error("Select --cd or --summarize")
    params = p2.load(ROOT / "config/reference.json")
    params["crossflowCd"] = args.cd
    contract = p2.load(ROOT / "config/p0-contract.json")
    original = p2.load(ROOT / "results/p2-p0-v1/single-collocation.json")["history"][4]["candidate"]
    path = DIRECTORY / f"cd-{args.cd:.1f}.json"
    history = p2.load(path)["history"] if path.exists() else []
    started = time.perf_counter()
    row = dict(seedSource="results/p2-p0-v1/single-collocation.json history[4]")
    print(f"START crossflowCd={args.cd}, same P2 local family, 140 s solver limit", flush=True)
    try:
        candidate = original if args.cd == 1.2 else p2.optimize(
            params, contract, p2.from_candidate(original, {}), .55, 400, shooting=True)
        row.update(candidate=candidate, replay={str(step): replay(candidate, args.cd, step) for step in (.005, .0025)})
    except (RuntimeError, subprocess.CalledProcessError) as error:
        row["error"] = str(error)
    row["elapsedSeconds"] = time.perf_counter() - started
    history.append(row)
    write_json(path, dict(crossflowCd=args.cd, casadi=p2.ca.__version__, history=history))
    print(f"DONE crossflowCd={args.cd}, accepted={accepted(row)}, seconds={row['elapsedSeconds']:.1f}", flush=True)
    if "candidate" in row:
        print(row["candidate"]["status"], row["candidate"]["propellantKg"], flush=True)
        for step, result in row["replay"].items():
            print(step, result["event"], result["violations"], result["touchdown"], flush=True)
    else:
        print(row["error"], flush=True)
    summarize()


if __name__ == "__main__":
    main()
