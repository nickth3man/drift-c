#!/usr/bin/env python3
"""Render telemetry as standalone SVG charts.

    python tools/telemetry/plot_telemetry.py TELEMETRY.csv --out artifacts/plots
    python tools/telemetry/plot_telemetry.py run.csv --baseline base.csv --out plots

One SVG per chart, no external libraries, no fonts to install. make_report.py inlines the
same chart definitions into a single HTML page; this script exists for when you want the
individual files (a PR comment, a bug report, a slide).
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import telemetry_common as tc  # noqa: E402

WHEEL_RADIUS_M = 0.31  # config.h WHEEL_RADIUS_M; only used to draw wheel speed in m/s

SVG_STYLE = """<style>
.chart { background: #14161a; }
.chart-title { fill: #e6e8ec; font: 600 13px sans-serif; }
.tick { fill: #9aa0aa; font: 10px sans-serif; }
.axis { fill: #9aa0aa; font: 11px sans-serif; }
.legend { fill: #d5d8dd; font: 10px sans-serif; }
.grid { stroke: #2b2f36; stroke-width: 1; }
</style>"""


def chart_definitions(run: tc.Run, baseline: Optional[tc.Run] = None):
    """(filename, title, x, series, x_label, y_label, equal_aspect) for every chart.

    Shared by this script and make_report.py so a plot cannot exist in one and not the other.
    """
    time = run.column("time_s")

    def series(*names: str) -> List[Tuple[str, Sequence[float]]]:
        out: List[Tuple[str, Sequence[float]]] = []
        for name in names:
            out.append((name, run.column(name)))
            if baseline is not None and baseline.has(name):
                out.append(("baseline " + name, baseline.column(name)))
        return out

    charts = [
        (
            "trajectory",
            "Trajectory (x forward, y left)",
            run.column("position_x_m"),
            (
                [("current", run.column("position_y_m"))]
                + ([("baseline", baseline.column("position_y_m"))] if baseline is not None else [])
            ),
            "x (m)",
            "y (m)",
            True,
        ),
        (
            "speed",
            "Speed and longitudinal velocity",
            time,
            series("speed_mps", "velocity_longitudinal_mps", "velocity_lateral_mps"),
            "time (s)",
            "m/s",
            False,
        ),
        (
            "steering_yaw",
            "Steering angle and yaw rate",
            time,
            series("steering_angle_rad", "yaw_rate_rad_s"),
            "time (s)",
            "rad, rad/s",
            False,
        ),
        ("sideslip", "Body sideslip", time, series("body_sideslip_rad"), "time (s)", "rad", False),
        (
            "slip_angles",
            "Axle slip angles",
            time,
            series("front_slip_angle_rad", "rear_slip_angle_rad"),
            "time (s)",
            "rad",
            False,
        ),
        (
            "slip_ratios",
            "Axle slip ratios",
            time,
            series("front_slip_ratio", "rear_slip_ratio"),
            "time (s)",
            "-",
            False,
        ),
        # WHEEL_RADIUS_M from config.h: the telemetry schema records wheel speed in rad/s,
        # and comparing it against ground speed is only meaningful in m/s.
        (
            "wheel_speed",
            "Wheel surface speed vs ground speed",
            time,
            [
                (
                    "front wheel surface",
                    [w * WHEEL_RADIUS_M for w in run.column("front_wheel_omega_rad_s")],
                ),
                (
                    "rear wheel surface",
                    [w * WHEEL_RADIUS_M for w in run.column("rear_wheel_omega_rad_s")],
                ),
                ("ground speed", run.column("speed_mps")),
            ],
            "time (s)",
            "m/s",
            False,
        ),
        (
            "normal_loads",
            "Axle normal loads",
            time,
            series("front_normal_load_n", "rear_normal_load_n"),
            "time (s)",
            "N",
            False,
        ),
        (
            "forces",
            "Limited tire forces",
            time,
            series(
                "front_fx_limited_n", "rear_fx_limited_n", "front_fy_limited_n", "rear_fy_limited_n"
            ),
            "time (s)",
            "N",
            False,
        ),
        (
            "friction_usage",
            "Friction ellipse usage",
            time,
            series("front_friction_usage", "rear_friction_usage"),
            "time (s)",
            "0..1",
            False,
        ),
        (
            "engine",
            "Engine speed and gear",
            time,
            series("engine_rpm", "selected_gear"),
            "time (s)",
            "rpm / gear",
            False,
        ),
        (
            "torques",
            "Drive and brake torques",
            time,
            series(
                "drive_torque_nm",
                "front_brake_torque_nm",
                "rear_brake_torque_nm",
                "handbrake_torque_nm",
            ),
            "time (s)",
            "N*m",
            False,
        ),
    ]

    # Phase 3 charts, present only when the run carries the Phase 3 columns — a Phase 2
    # baseline without them should not sprout charts of synthesized zeros.
    if run.has("load_transfer_n"):
        charts += [
            (
                "axle_loads",
                "Axle loads: static split vs dynamic",
                time,
                series(
                    "static_front_load_n",
                    "static_rear_load_n",
                    "dynamic_front_load_n",
                    "dynamic_rear_load_n",
                ),
                "time (s)",
                "N",
                False,
            ),
            (
                "load_transfer",
                "Longitudinal load transfer (positive = rearward)",
                time,
                series("load_transfer_n"),
                "time (s)",
                "N",
                False,
            ),
            (
                "accel_filter",
                "Acceleration filter: previous -> filtered -> solved",
                time,
                series(
                    "previous_long_accel_mps2", "filtered_long_accel_mps2", "solved_long_accel_mps2"
                ),
                "time (s)",
                "m/s^2",
                False,
            ),
            (
                "drag_vs_speed",
                "Aerodynamic drag against speed",
                run.column("speed_mps"),
                [("aero drag", run.column("aero_drag_n"))]
                + (
                    [("baseline aero drag", baseline.column("aero_drag_n"))]
                    if baseline is not None and baseline.has("aero_drag_n")
                    else []
                ),
                "speed (m/s)",
                "N",
                False,
            ),
            (
                "resistance",
                "Resistance forces over the run",
                time,
                series("aero_drag_n", "rolling_resistance_n"),
                "time (s)",
                "N",
                False,
            ),
            (
                "driver_inputs",
                "Driver inputs",
                time,
                series("throttle_input", "brake_input", "handbrake_input"),
                "time (s)",
                "0..1",
                False,
            ),
            (
                "lateral_accel",
                "Lateral acceleration",
                time,
                series("lateral_accel_mps2"),
                "time (s)",
                "m/s^2",
                False,
            ),
        ]
    return charts


def render_chart(definition) -> str:
    _, title, x, series, x_label, y_label, equal_aspect = definition
    return tc.line_chart(
        title, x, series, x_label=x_label, y_label=y_label, equal_aspect=equal_aspect
    )


def standalone_svg(svg: str) -> str:
    """Inline the stylesheet inside the root element so the file renders on its own."""
    insert_at = svg.find(">") + 1
    return svg[:insert_at] + SVG_STYLE + svg[insert_at:]


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("csv")
    parser.add_argument("--baseline")
    parser.add_argument("--out", default="artifacts/plots")
    args = parser.parse_args(argv)

    run = tc.load_run(args.csv)
    baseline = tc.load_run(args.baseline) if args.baseline else None

    os.makedirs(args.out, exist_ok=True)
    written = 0
    for definition in chart_definitions(run, baseline):
        svg = render_chart(definition)
        path = os.path.join(args.out, "%s_%s.svg" % (run.name, definition[0]))
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(standalone_svg(svg))
        written += 1

    print("wrote %d charts to %s" % (written, args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
