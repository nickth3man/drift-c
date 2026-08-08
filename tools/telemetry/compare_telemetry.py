#!/usr/bin/env python3
"""Tolerance-aware comparison of two telemetry runs.

    python tools/telemetry/compare_telemetry.py BASELINE.csv CURRENT.csv
    python tools/telemetry/compare_telemetry.py BASELINE.csv CURRENT.csv --markdown summary.md
    python tools/telemetry/compare_telemetry.py --baseline-dir DIR --current-dir DIR

Exit status is 0 when every comparison is inside tolerance and 1 when anything breached, so
this is usable directly as a CI gate.

Three comparison classes, deliberately kept apart:

  exact      tick count, gear sequence, wheel-lock transitions — a difference is a difference
  tolerance  positions, velocities, forces, loads, slip angles — float noise is not a failure
  derived    0-100 km/h, peak sideslip, steady radius, recovery time — what a human asks about

The state checksum is reported but never gates: it is only meaningful between two runs of the
same binary on the same toolchain, and CI compares across compilers.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import telemetry_common as tc  # noqa: E402


def compare_pair(baseline_path: str, current_path: str, quiet: bool = False):
    baseline = tc.load_run(baseline_path)
    current = tc.load_run(current_path)

    differences: List[tc.Difference] = []
    differences.extend(tc.compare_exact(baseline, current))
    differences.extend(tc.compare_columns(baseline, current))
    differences.extend(tc.compare_metrics(baseline, current))

    breaches = [d for d in differences if d.breached]

    if not quiet:
        print("=== %s  vs  %s" % (baseline_path, current_path))
        print(
            "    rows %d -> %d, checksum %s"
            % (
                len(baseline),
                len(current),
                "identical"
                if tc.checksum_matches(baseline, current)
                else "differs (informational)",
            )
        )
        metrics_base = tc.derived_metrics(baseline)
        metrics_current = tc.derived_metrics(current)
        print("    %-24s %14s %14s %12s" % ("metric", "baseline", "current", "delta"))
        for name in sorted(metrics_base):
            a, b = metrics_base[name], metrics_current.get(name)
            if a is None or b is None:
                print("    %-24s %14s %14s %12s" % (name, a, b, "-"))
            else:
                print("    %-24s %14.4f %14.4f %12.4f" % (name, a, b, b - a))
        if breaches:
            print("    breaches:")
            for difference in breaches:
                print("      ! %s" % difference.describe())
        else:
            print("    no breaches")
        print()

    return differences, breaches


def markdown_table(rows, headers) -> str:
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(str(cell) for cell in row) + " |")
    return "\n".join(lines)


def write_markdown(path: str, results) -> None:
    lines = ["## Physics regression", ""]
    rows = []
    for name, breaches, largest in results:
        status = "FAIL" if breaches else "PASS"
        rows.append([name, status, largest])
    lines.append(markdown_table(rows, ["Scenario", "Result", "Largest change"]))
    lines.append("")
    lines.append(
        "Tolerance-aware comparison; the state checksum is informational because CI builds "
        "with more than one compiler."
    )
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def largest_change_text(differences) -> str:
    """The single most interesting number for a PR summary line."""
    metrics = [d for d in differences if d.kind == "metric" and d.delta is not None]
    if not metrics:
        return "-"

    def severity(difference):
        if difference.tolerance in (None, 0):
            return difference.delta
        return difference.delta / difference.tolerance

    worst = max(metrics, key=severity)
    if worst.baseline in (None, 0) or abs(worst.baseline) < 1e-9:
        return "%s: %+.4g" % (worst.name, worst.current - worst.baseline)
    percent = 100.0 * (worst.current - worst.baseline) / abs(worst.baseline)
    return "%s: %+.2f%%" % (worst.name, percent)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("baseline", nargs="?", help="baseline CSV")
    parser.add_argument("current", nargs="?", help="current CSV")
    parser.add_argument("--baseline-dir", help="compare every baseline CSV in this directory")
    parser.add_argument("--current-dir", help="directory holding the matching current CSVs")
    parser.add_argument("--markdown", help="write a Markdown summary here")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    results = []
    failed = False

    if args.baseline_dir:
        if not args.current_dir:
            parser.error("--baseline-dir needs --current-dir")
        names = sorted(f for f in os.listdir(args.baseline_dir) if f.endswith(".csv"))
        if not names:
            print("no baseline CSVs in %s" % args.baseline_dir)
            return 0
        for name in names:
            baseline_path = os.path.join(args.baseline_dir, name)
            current_path = os.path.join(args.current_dir, name)
            if not os.path.exists(current_path):
                print("MISSING: %s has no counterpart in %s" % (name, args.current_dir))
                results.append((name, ["missing current run"], "-"))
                failed = True
                continue
            differences, breaches = compare_pair(baseline_path, current_path, args.quiet)
            results.append((name, breaches, largest_change_text(differences)))
            failed = failed or bool(breaches)
    else:
        if not args.baseline or not args.current:
            parser.error("give two CSV paths, or --baseline-dir with --current-dir")
        differences, breaches = compare_pair(args.baseline, args.current, args.quiet)
        results.append(
            (os.path.basename(args.baseline), breaches, largest_change_text(differences))
        )
        failed = bool(breaches)

    if args.markdown:
        write_markdown(args.markdown, results)
        print("wrote %s" % args.markdown)

    print("%d comparison(s), %s" % (len(results), "FAIL" if failed else "PASS"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
