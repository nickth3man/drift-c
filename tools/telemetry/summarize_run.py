#!/usr/bin/env python3
"""Summarise one telemetry run as text or JSON.

    python tools/telemetry/summarize_run.py artifacts/telemetry/scenario_skidpad.csv
    python tools/telemetry/summarize_run.py artifacts/telemetry/*.csv --json artifacts/summary.json

Derived metrics only — the numbers a human asks about after a change ("did it still hit 100
in the same time? did the peak sideslip move?"). The raw columns are what compare_telemetry
looks at; this is what goes in a commit message.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import telemetry_common as tc  # noqa: E402


def summarize(path: str) -> dict:
    run = tc.load_run(path)
    metrics = tc.derived_metrics(run)
    summary = {
        "path": path,
        "name": run.name,
        "rows": len(run),
        "metrics": metrics,
        "gear_sequence": tc.gear_sequence(run),
        "lock_events": [
            {"tick": tick, "axle": axle, "state": state}
            for tick, axle, state in tc.lock_events(run)
        ],
    }
    return summary


def print_summary(summary: dict) -> None:
    print("%s  (%d rows)" % (summary["path"], summary["rows"]))
    for name in sorted(summary["metrics"]):
        value = summary["metrics"][name]
        if value is None:
            print("    %-24s -" % name)
        elif name == "final_checksum":
            print("    %-24s %08x" % (name, int(value)))
        else:
            print("    %-24s %.6g" % (name, value))
    print("    %-24s %s" % ("gear sequence", summary["gear_sequence"]))
    print("    %-24s %d" % ("wheel lock transitions", len(summary["lock_events"])))
    print()


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("csv", nargs="+")
    parser.add_argument("--json", help="write the summaries here as JSON")
    args = parser.parse_args(argv)

    summaries = []
    for path in args.csv:
        summary = summarize(path)
        summaries.append(summary)
        print_summary(summary)

    if args.json:
        directory = os.path.dirname(os.path.abspath(args.json))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(summaries if len(summaries) > 1 else summaries[0], handle, indent=2)
        print("wrote %s" % args.json)

    return 0


if __name__ == "__main__":
    sys.exit(main())
