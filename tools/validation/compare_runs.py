#!/usr/bin/env python3
"""compare_runs.py — Compare two validation runs or two validation suites.

Usage:
    python tools/validation/compare_runs.py artifacts/validation/latest \
        artifacts/validation/20260807-120000-chicane_v1-abc1234
    python tools/validation/compare_runs.py artifacts/validation/latest/rwd_grip \
        artifacts/validation/previous/rwd_grip
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List


def load_json(path: str) -> Dict[str, Any] | None:
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"WARNING: failed to load '{path}': {e}")
        return None


def compare_single_run(path_a: str, path_b: str, quiet: bool = False) -> Dict[str, Any]:
    json_a = (
        load_json(os.path.join(path_a, "run.json")) if os.path.isdir(path_a) else load_json(path_a)
    )
    json_b = (
        load_json(os.path.join(path_b, "run.json")) if os.path.isdir(path_b) else load_json(path_b)
    )

    car_a = json_a.get("car", {}).get("id", "unknown") if json_a else "unknown"
    car_b = json_b.get("car", {}).get("id", "unknown") if json_b else "unknown"

    status_a = json_a.get("result", {}).get("status", "MISSING") if json_a else "MISSING"
    status_b = json_b.get("result", {}).get("status", "MISSING") if json_b else "MISSING"

    time_a = json_a.get("lap", {}).get("timed_lap_time_s", 0.0) if json_a else 0.0
    time_b = json_b.get("lap", {}).get("timed_lap_time_s", 0.0) if json_b else 0.0
    time_delta = time_b - time_a

    metrics_a = json_a.get("metrics", {}) if json_a else {}
    metrics_b = json_b.get("metrics", {}) if json_b else {}

    metric_deltas: Dict[str, float] = {}
    for key in set(metrics_a.keys()).union(metrics_b.keys()):
        val_a = metrics_a.get(key)
        val_b = metrics_b.get(key)
        if (
            isinstance(val_a, (int, float))
            and isinstance(val_b, (int, float))
            and not isinstance(val_a, bool)
        ):
            metric_deltas[key] = val_b - val_a

    return {
        "car": car_a if car_a != "unknown" else car_b,
        "status_a": status_a,
        "status_b": status_b,
        "time_a": time_a,
        "time_b": time_b,
        "time_delta": time_delta,
        "metric_deltas": metric_deltas,
    }


def compare_suites(dir_a: str, dir_b: str, quiet: bool = False) -> int:
    suite_a_path = os.path.join(dir_a, "suite.json")
    suite_b_path = os.path.join(dir_b, "suite.json")

    json_a = load_json(suite_a_path)
    json_b = load_json(suite_b_path)

    # Key by case, not by car: the suite runs every car on several tracks and from several
    # start gates, so car_id alone collapses three distinct runs onto one row.
    def index(suite: Dict[str, Any] | None) -> Dict[str, Dict[str, Any]]:
        if not suite:
            return {}
        return {r.get("case_id", r["car_id"]): r for r in suite.get("runs", [])}

    cars_a = index(json_a)
    cars_b = index(json_b)

    def run_dir(base: str, entry: Dict[str, Any] | None, key: str) -> str:
        # Runs live at <suite>/<case_id>/<car_id>/run.json; the suite records that relative
        # path, so follow it rather than guessing the layout.
        rel = entry.get("run_json") if entry else None
        if rel:
            return os.path.join(base, os.path.dirname(rel))
        return os.path.join(base, key)

    all_cars = sorted(set(cars_a.keys()).union(cars_b.keys()))
    if not all_cars:
        # Fallback to subdirectories if no suite.json exists
        sub_a = (
            [d for d in os.listdir(dir_a) if os.path.isdir(os.path.join(dir_a, d))]
            if os.path.exists(dir_a)
            else []
        )
        sub_b = (
            [d for d in os.listdir(dir_b) if os.path.isdir(os.path.join(dir_b, d))]
            if os.path.exists(dir_b)
            else []
        )
        all_cars = sorted(set(sub_a).union(sub_b))

    print(f"Comparing Validation Suites / Runs:\n  A: {dir_a}\n  B: {dir_b}\n")
    print(
        f"{'CASE':<28} {'STATUS A':<10} {'STATUS B':<10} "
        f"{'LAP A (s)':<12} {'LAP B (s)':<12} {'DELTA (s)':<10}"
    )
    print("-" * 84)

    regressions = 0

    for car in all_cars:
        run_a_path = run_dir(dir_a, cars_a.get(car), car)
        run_b_path = run_dir(dir_b, cars_b.get(car), car)

        diff = compare_single_run(run_a_path, run_b_path, quiet)

        status_a = diff["status_a"]
        status_b = diff["status_b"]
        time_a = diff["time_a"]
        time_b = diff["time_b"]
        delta = diff["time_delta"]

        status_flag = ""
        if status_a == "PASS" and status_b != "PASS":
            status_flag = " [REGRESSION]"
            regressions += 1
        elif status_a != "PASS" and status_b == "PASS":
            status_flag = " [IMPROVED]"
        elif status_a == "MISSING" and status_b == "MISSING":
            # Neither side resolved to a run.json. Silently reporting PASS here is how a broken
            # path convention turns into a green comparison that compared nothing.
            status_flag = " [NOT FOUND]"
            regressions += 1

        print(
            f"{car:<28} {status_a:<10} {status_b:<10} "
            f"{time_a:<12.3f} {time_b:<12.3f} {delta:<+10.3f}{status_flag}"
        )

    print("-" * 84)
    if regressions > 0:
        print(f"RESULT: FAIL ({regressions} case regression(s) detected)")
        return 1
    print("RESULT: PASS")
    return 0


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dir_a", help="first validation suite/run directory")
    parser.add_argument("dir_b", help="second validation suite/run directory")
    parser.add_argument("--quiet", action="store_true", help="suppress detailed metrics")
    args = parser.parse_args(argv)

    return compare_suites(args.dir_a, args.dir_b, args.quiet)


if __name__ == "__main__":
    sys.exit(main())
