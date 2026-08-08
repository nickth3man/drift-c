#!/usr/bin/env python3
"""run_suite.py — Validation suite orchestrator for Drifty Milestone 1.

Runs `drifty.exe --validate-lap --car ID --track NAME --start-checkpoint N` for every roster car
and each validation scenario, aggregates individual `run.json` files into a top-level `suite.json`,
copies results to `artifacts/validation/latest/`, and exits nonzero if any case failed.

Usage:
    python tools/validation/run_suite.py
    python tools/validation/run_suite.py --no-video
    python tools/validation/run_suite.py --cars rwd_grip,fwd_light
    python tools/validation/run_suite.py --exe build/dev/drifty.exe --out artifacts/validation
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from typing import Any, Dict, List

# Laps each case drives: an out-lap plus three timed ones. Must match VALIDATION_RUN_LAPS in
# src/game/validation_metrics.h — the C side decides how many laps are driven, this side only
# checks that the report proves they all happened.
RUN_LAPS = 4


def get_commit(repo_root: str) -> str:
    try:
        res = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            check=True,
        )
        return res.stdout.strip()
    except Exception:
        return "unknown"


def list_roster_cars(exe_path: str, repo_root: str) -> List[str]:
    res = subprocess.run([exe_path, "--list-cars"], cwd=repo_root, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"Failed to list cars from {exe_path}: {res.stderr}")
    raw_lines = [line.strip() for line in res.stdout.splitlines() if line.strip()]
    cars = [
        c
        for c in raw_lines
        if not c.startswith(
            (
                "INFO:",
                "WARNING:",
                "ERROR:",
                "GL:",
                "DISPLAY:",
                "SYSTEM:",
                "TIMER:",
                "AUDIO:",
                "GAME:",
                "TEXTURE:",
                "SHADER:",
                "RLGL:",
                "FONT:",
                "FBO:",
                "FILEIO:",
                "PLATFORM:",
            )
        )
        and " " not in c
        and ":" not in c
    ]
    if not cars:
        return ["rwd_grip", "rwd_power", "fwd_light", "fwd_hot", "awd_rally", "awd_gt"]
    return cars


def run_car_validation(
    exe_path: str,
    car_id: str,
    case_dir: str,
    track: str,
    start_checkpoint: int,
    no_video: bool,
    repo_root: str,
) -> Dict[str, Any]:
    cmd = [
        exe_path,
        "--validate-lap",
        "--car",
        car_id,
        "--track",
        track,
        "--start-checkpoint",
        str(start_checkpoint),
        "--output",
        case_dir,
    ]
    if no_video:
        cmd.append("--no-video")

    proc = subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True)
    json_path = os.path.join(case_dir, car_id, "run.json")
    if os.path.exists(json_path):
        try:
            with open(json_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            return {
                "car_id": car_id,
                "result": {"status": "FAIL", "failure_reason": f"corrupt_json: {e}"},
                "run_json": json_path,
            }

    return {
        "car_id": car_id,
        "track": track,
        "start_checkpoint": start_checkpoint,
        "result": {
            "status": "FAIL",
            "failure_reason": f"exit_{proc.returncode}: {proc.stderr.strip()}",
        },
        "run_json": json_path,
    }


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", default="build/dev/drifty.exe", help="path to drifty executable")
    parser.add_argument("--out", default="artifacts/validation", help="root output directory")
    parser.add_argument("--cars", help="comma-separated car IDs filter")
    parser.add_argument("--no-video", action="store_true", help="disable MP4 video encoding")
    args = parser.parse_args(argv)

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    exe_path = os.path.join(repo_root, args.exe) if not os.path.isabs(args.exe) else args.exe

    if not os.path.exists(exe_path):
        print(f"ERROR: executable not found at '{exe_path}'. Run build.bat first.")
        return 1

    try:
        cars = list_roster_cars(exe_path, repo_root)
    except Exception as e:
        print(f"ERROR: {e}")
        return 1

    if args.cars:
        # Preserve requested IDs verbatim so an invalid ID becomes a deliberate failed case
        # instead of being silently filtered out of the suite.
        cars = [c.strip() for c in args.cars.split(",") if c.strip()]

    scenarios = [("chicane", 0), ("sprint", 3), ("technical", 3)]
    cases = [(car_id, track, start) for track, start in scenarios for car_id in cars]

    stamp = time.strftime("%Y%m%d-%H%M%S")
    commit = get_commit(repo_root)
    suite_id = f"{stamp}-track-suite_v2-{commit}"

    out_root = os.path.join(repo_root, args.out) if not os.path.isabs(args.out) else args.out
    suite_dir = os.path.join(out_root, suite_id)
    latest_dir = os.path.join(out_root, "latest")

    os.makedirs(suite_dir, exist_ok=True)

    print(f"Running validation suite '{suite_id}' for {len(cases)} case(s)...")

    results: List[Dict[str, Any]] = []
    passed_count = 0
    failed_count = 0

    for car_id, track, start_checkpoint in cases:
        case_id = f"{track}-start{start_checkpoint}-{car_id}"
        case_dir = os.path.join(suite_dir, case_id)
        print(f"  -> Validating car '{car_id}' on '{track}' from checkpoint {start_checkpoint}...")
        run_data = run_car_validation(
            exe_path,
            car_id,
            case_dir,
            track,
            start_checkpoint,
            args.no_video,
            repo_root,
        )
        status = run_data.get("result", {}).get("status", "FAIL")
        reason = run_data.get("result", {}).get("failure_reason")
        lap_data = run_data.get("lap", {})
        timed_time = lap_data.get("timed_lap_time_s", 0.0)
        best_time = lap_data.get("best_timed_lap_time_s", 0.0)
        mean_time = lap_data.get("mean_timed_lap_time_s", 0.0)
        # A process-level PASS is not enough: assert the report proves every lap of the run was
        # completed from this scenario's start gate, preventing a partial lap from being counted.
        track_data = run_data.get("track", {})
        expected_crossings = RUN_LAPS * int(track_data.get("checkpoint_count", 0))
        actual_crossings = int(run_data.get("lap", {}).get("checkpoints_passed", 0))
        if status == "PASS" and (
            actual_crossings != expected_crossings
            or track_data.get("start_checkpoint_index") != start_checkpoint
        ):
            status = "FAIL"
            reason = "checkpoint_missed"

        if status == "PASS":
            passed_count += 1
            print(
                f"     PASS (best {best_time:.3f} s, mean {mean_time:.3f} s over "
                f"{RUN_LAPS - 1} timed laps)"
            )
        else:
            failed_count += 1
            print(f"     FAIL ({reason})")

        results.append(
            {
                "case_id": case_id,
                "car_id": car_id,
                "track": track,
                "start_checkpoint": start_checkpoint,
                "status": status,
                "failure_reason": reason,
                "timed_lap_time_s": timed_time,
                "best_timed_lap_time_s": best_time,
                "mean_timed_lap_time_s": mean_time,
                "run_json": f"{case_id}/{car_id}/run.json",
                "full_data": run_data,
            }
        )

    suite_summary = {
        "schema_version": "1.0.0",
        "suite_id": suite_id,
        "scenarios": [{"track": track, "start_checkpoint": start} for track, start in scenarios],
        "commit": commit,
        "total": len(cases),
        "passed": passed_count,
        "failed": failed_count,
        "runs": [
            {
                "case_id": r["case_id"],
                "car_id": r["car_id"],
                "track": r["track"],
                "start_checkpoint": r["start_checkpoint"],
                "status": r["status"],
                "failure_reason": r["failure_reason"],
                "timed_lap_time_s": r["timed_lap_time_s"],
                "best_timed_lap_time_s": r["best_timed_lap_time_s"],
                "mean_timed_lap_time_s": r["mean_timed_lap_time_s"],
                "run_json": r["run_json"],
            }
            for r in results
        ],
    }

    suite_json_path = os.path.join(suite_dir, "suite.json")
    with open(suite_json_path, "w", encoding="utf-8") as f:
        json.dump(suite_summary, f, indent=2)

    # Copy suite artifacts to artifacts/validation/latest/
    if os.path.exists(latest_dir):
        shutil.rmtree(latest_dir)
    shutil.copytree(suite_dir, latest_dir)

    print(f"\nSuite complete: {passed_count}/{len(cases)} passed ({failed_count} failed).")
    print(f"Summary written to: {suite_json_path}")
    print(f"Latest suite copied to: {latest_dir}")

    return 0 if failed_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
