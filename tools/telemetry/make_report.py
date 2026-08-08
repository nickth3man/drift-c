#!/usr/bin/env python3
"""Build one self-contained HTML report for a telemetry run.

    python tools/telemetry/make_report.py TELEMETRY.csv --out artifacts/report.html
    python tools/telemetry/make_report.py current.csv --baseline tests/baselines/skidpad.csv
    python tools/telemetry/make_report.py --dir artifacts/telemetry --out artifacts/report.html

Everything is inlined — charts are SVG elements, styles are a <style> block, there is no
script and no external request. Open it from disk, attach it to a PR, or drop it in a
failure bundle; it works the same in all three.

With --baseline, every chart also draws the baseline and a comparison table appears at the
top with the tolerance-aware verdict for each metric.
"""

from __future__ import annotations

import argparse
import datetime
import html
import os
import sys
from typing import List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import plot_telemetry  # noqa: E402
import telemetry_common as tc  # noqa: E402

PAGE_STYLE = """
:root { color-scheme: dark; }
body { margin: 0; padding: 24px; background: #0f1114; color: #e6e8ec;
       font: 14px/1.5 -apple-system, Segoe UI, Roboto, sans-serif; }
h1 { font-size: 20px; margin: 0 0 4px; }
h2 { font-size: 15px; margin: 28px 0 8px; color: #cfd3da; }
.sub { color: #9aa0aa; font-size: 12px; margin-bottom: 20px; }
table { border-collapse: collapse; margin: 8px 0 20px; font-size: 13px; width: 100%;
        max-width: 900px; }
th, td { text-align: left; padding: 5px 10px; border-bottom: 1px solid #23262c; }
th { color: #9aa0aa; font-weight: 600; }
td.num { text-align: right; font-variant-numeric: tabular-nums; }
.pass { color: #8ce99a; }
.fail { color: #ff8b80; font-weight: 600; }
.info { color: #9aa0aa; }
.grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(420px, 1fr)); gap: 14px; }
.card { background: #14161a; border: 1px solid #23262c; border-radius: 6px; padding: 6px; }
.banner { background: #2a1416; border: 1px solid #6d2b28; border-radius: 6px; padding: 10px 14px;
          margin-bottom: 18px; }
.banner.ok { background: #142a1a; border-color: #2c6d3f; }
code { background: #1a1d22; padding: 1px 5px; border-radius: 3px; }
.chart { background: #14161a; }
.chart-title { fill: #e6e8ec; font: 600 13px sans-serif; }
.tick { fill: #9aa0aa; font: 10px sans-serif; }
.axis { fill: #9aa0aa; font: 11px sans-serif; }
.legend { fill: #d5d8dd; font: 10px sans-serif; }
svg .grid { stroke: #2b2f36; stroke-width: 1; }
"""


def format_value(value, name: str = "") -> str:
    if value is None:
        return "-"
    if name == "final_checksum":
        return "%08x" % int(value)
    if isinstance(value, float):
        return "%.6g" % value
    return html.escape(str(value))


def metrics_table(run: tc.Run, baseline: Optional[tc.Run]) -> str:
    metrics = tc.derived_metrics(run)
    base_metrics = tc.derived_metrics(baseline) if baseline is not None else {}
    differences = {d.name: d for d in tc.compare_metrics(baseline, run)} if baseline else {}

    rows = []
    for name in sorted(metrics):
        current_value = metrics[name]
        cells = [
            "<td>%s</td>" % html.escape(name),
            '<td class="num">%s</td>' % format_value(current_value, name),
        ]
        if baseline is not None:
            base_value = base_metrics.get(name)
            cells.append('<td class="num">%s</td>' % format_value(base_value, name))
            difference = differences.get(name)
            if difference is None or difference.delta is None:
                cells.append('<td class="num info">-</td><td class="info">-</td>')
            else:
                verdict = (
                    '<span class="fail">REVIEW</span>'
                    if difference.breached
                    else '<span class="pass">ok</span>'
                )
                cells.append(
                    '<td class="num">%+.6g</td><td>%s</td>'
                    % (difference.current - difference.baseline, verdict)
                )
        rows.append("<tr>%s</tr>" % "".join(cells))

    headers = ["Metric", "Current"]
    if baseline is not None:
        headers += ["Baseline", "Delta", "Verdict"]
    return "<table><thead><tr>%s</tr></thead><tbody>%s</tbody></table>" % (
        "".join("<th>%s</th>" % h for h in headers),
        "".join(rows),
    )


def column_table(run: tc.Run, baseline: Optional[tc.Run]) -> str:
    if baseline is None:
        return ""
    differences = [d for d in tc.compare_columns(baseline, run) if d.kind == "column"]
    differences.sort(key=lambda d: (d.delta or 0.0) - (d.tolerance or 0.0), reverse=True)
    rows = []
    for difference in differences[:14]:
        verdict = (
            '<span class="fail">breach</span>'
            if difference.breached
            else '<span class="pass">ok</span>'
        )
        rows.append(
            '<tr><td>%s</td><td class="num">%.6g</td><td class="num">%.6g</td>'
            '<td class="num">%s</td><td>%s</td></tr>'
            % (
                html.escape(difference.name),
                difference.delta or 0.0,
                difference.tolerance or 0.0,
                "-" if difference.index is None else difference.index,
                verdict,
            )
        )
    return (
        "<table><thead><tr><th>Column</th><th>Worst |delta|</th><th>Tolerance</th>"
        "<th>Row</th><th>Verdict</th></tr></thead><tbody>%s</tbody></table>" % "".join(rows)
    )


def exact_table(run: tc.Run, baseline: Optional[tc.Run]) -> str:
    if baseline is None:
        return ""
    rows = []
    for difference in tc.compare_exact(baseline, run):
        verdict = (
            '<span class="fail">differs</span>'
            if difference.breached
            else '<span class="pass">identical</span>'
        )
        rows.append(
            "<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>"
            % (
                html.escape(difference.name),
                format_value(difference.baseline),
                format_value(difference.current),
                verdict,
            )
        )
    checksum = (
        '<span class="pass">identical</span>'
        if tc.checksum_matches(baseline, run)
        else '<span class="info">differs (informational across toolchains)</span>'
    )
    rows.append("<tr><td>state checksum</td><td>-</td><td>-</td><td>%s</td></tr>" % checksum)
    return (
        "<table><thead><tr><th>Exact comparison</th><th>Baseline</th><th>Current</th>"
        "<th>Verdict</th></tr></thead><tbody>%s</tbody></table>" % "".join(rows)
    )


def failures_section(failures_path: Optional[str]) -> str:
    """Render failure.txt / summary.json from a failure bundle, when one was passed."""
    if not failures_path or not os.path.exists(failures_path):
        return ""
    with open(failures_path, "r", encoding="utf-8", errors="replace") as handle:
        text = handle.read()
    return '<h2>Failing assertions</h2><div class="banner"><pre>%s</pre></div>' % html.escape(
        text.strip()
    )


def build_report(
    runs: List[tc.Run], baseline: Optional[tc.Run], failures: Optional[str], title: str
) -> str:
    generated = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    parts: List[str] = []
    parts.append('<!doctype html><html><head><meta charset="utf-8">')
    parts.append('<meta name="viewport" content="width=device-width, initial-scale=1">')
    parts.append(
        "<title>%s</title><style>%s</style></head><body>" % (html.escape(title), PAGE_STYLE)
    )
    parts.append("<h1>%s</h1>" % html.escape(title))
    parts.append(
        '<div class="sub">generated %s &middot; %d run(s)%s</div>'
        % (
            generated,
            len(runs),
            " &middot; baseline <code>%s</code>" % html.escape(baseline.path)
            if baseline is not None
            else "",
        )
    )

    parts.append(failures_section(failures))

    for run in runs:
        # A baseline applies to a run when it is the only run, or when the file names match —
        # comparing a skidpad against a handbrake baseline would be worse than no comparison.
        reference: Optional[tc.Run] = None
        if baseline is not None and (len(runs) == 1 or run.name == baseline.name):
            reference = baseline

        breaches = []
        if reference is not None:
            breaches = [d for d in tc.compare_metrics(reference, run) if d.breached]
            breaches += [d for d in tc.compare_columns(reference, run) if d.breached]
        banner_class = "banner ok" if not breaches else "banner"
        if reference is not None:
            summary = (
                "no tolerance breaches against %s" % os.path.basename(reference.path)
                if not breaches
                else "%d comparison(s) outside tolerance" % len(breaches)
            )
            parts.append(
                '<div class="%s">%s: %s</div>'
                % (banner_class, html.escape(run.name), html.escape(summary))
            )

        parts.append("<h2>%s &mdash; derived metrics</h2>" % html.escape(run.name))
        parts.append(metrics_table(run, reference))
        if reference is not None:
            parts.append("<h2>Exact comparisons</h2>")
            parts.append(exact_table(run, reference))
            parts.append("<h2>Largest column deviations</h2>")
            parts.append(column_table(run, reference))

        parts.append("<h2>%s &mdash; charts</h2>" % html.escape(run.name))
        parts.append('<div class="grid">')
        for definition in plot_telemetry.chart_definitions(run, reference):
            parts.append('<div class="card">%s</div>' % plot_telemetry.render_chart(definition))
        parts.append("</div>")

    parts.append("</body></html>")
    return "".join(parts)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("csv", nargs="*", help="telemetry CSVs to include")
    parser.add_argument("--dir", help="include every CSV in this directory")
    parser.add_argument("--baseline", help="baseline CSV to compare against")
    parser.add_argument("--failures", help="failure.txt from a failure bundle")
    parser.add_argument("--title", default="Drifty telemetry report")
    parser.add_argument("--out", default="artifacts/report.html")
    args = parser.parse_args(argv)

    paths = list(args.csv)
    if args.dir:
        paths += [
            os.path.join(args.dir, f) for f in sorted(os.listdir(args.dir)) if f.endswith(".csv")
        ]
    if not paths:
        parser.error("give at least one CSV, or --dir")

    runs = [tc.load_run(path) for path in paths]
    baseline = tc.load_run(args.baseline) if args.baseline else None

    directory = os.path.dirname(os.path.abspath(args.out))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as handle:
        handle.write(build_report(runs, baseline, args.failures, args.title))

    print(
        "wrote %s (%d run(s), %d chart(s) each)"
        % (args.out, len(runs), len(plot_telemetry.chart_definitions(runs[0], baseline)))
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
