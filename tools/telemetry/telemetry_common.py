"""Shared telemetry loading, derived metrics, tolerance-aware comparison, and SVG plotting.

Standard library only, on purpose. These tools run in CI, on a fresh Windows checkout, and
inside a failure triage session; none of those should need a package install first.

The CSV schema is whatever `telemetry_header()` in src/game/telemetry.c writes — the header
row is parsed rather than assumed, so adding a column here is not a code change.
"""

from __future__ import annotations

import csv
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

FIXED_HZ = 120.0

# ---------------------------------------------------------------------------- loading --


class Run:
    """One telemetry CSV: columns by name, plus the comment lines above the header."""

    def __init__(self, path: str, columns: Dict[str, List[float]], meta: List[str]):
        self.path = path
        self.name = os.path.splitext(os.path.basename(path))[0]
        self.columns = columns
        self.meta = meta

    def __len__(self) -> int:
        first = next(iter(self.columns.values()), [])
        return len(first)

    def has(self, name: str) -> bool:
        return name in self.columns

    def column(self, name: str, default: float = 0.0) -> List[float]:
        if name in self.columns:
            return self.columns[name]
        return [default] * len(self)

    def value(self, name: str, index: int, default: float = 0.0) -> float:
        column = self.columns.get(name)
        if column is None or index >= len(column):
            return default
        return column[index]


def load_run(path: str) -> Run:
    """Read a telemetry CSV. Lines starting with '#' before the header are kept as metadata."""
    meta: List[str] = []
    columns: Dict[str, List[float]] = {}
    order: List[str] = []

    with open(path, "r", newline="", encoding="utf-8") as handle:
        rows = csv.reader(handle)
        header: Optional[List[str]] = None
        for row in rows:
            if not row:
                continue
            if row[0].startswith("#"):
                meta.append(",".join(row))
                continue
            if header is None:
                header = [name.strip() for name in row]
                order = header
                columns = {name: [] for name in header}
                continue
            if len(row) != len(header):
                # A short final line means the writer was interrupted; stop cleanly rather
                # than inventing values for the missing columns.
                break
            for name, text in zip(order, row):
                try:
                    columns[name].append(float(text))
                except ValueError:
                    columns[name].append(float("nan"))

    if header is None:
        raise ValueError("%s has no header row" % path)
    return Run(path, columns, meta)


# ------------------------------------------------------------------- derived metrics --


def _first_index_where(values: Sequence[float], predicate) -> Optional[int]:
    for index, value in enumerate(values):
        if predicate(value):
            return index
    return None


def _peak(values: Sequence[float]) -> float:
    return max((abs(v) for v in values if not math.isnan(v)), default=0.0)


def derived_metrics(run: Run) -> Dict[str, Optional[float]]:
    """Physically meaningful summaries of a run.

    These are what a human actually compares between two builds; a row-by-row diff of 40
    columns tells you that something changed, not what.
    """
    speed = run.column("speed_mps")
    time = run.column("time_s")
    metrics: Dict[str, Optional[float]] = {}

    metrics["ticks"] = float(len(run))
    metrics["duration_s"] = time[-1] if time else 0.0
    metrics["peak_speed_mps"] = max(speed) if speed else 0.0
    metrics["peak_sideslip_rad"] = _peak(run.column("body_sideslip_rad"))
    metrics["peak_yaw_rate_rad_s"] = _peak(run.column("yaw_rate_rad_s"))
    metrics["peak_front_slip_rad"] = _peak(run.column("front_slip_angle_rad"))
    metrics["peak_rear_slip_rad"] = _peak(run.column("rear_slip_angle_rad"))
    metrics["peak_friction_usage"] = max(
        _peak(run.column("front_friction_usage")), _peak(run.column("rear_friction_usage"))
    )

    # 0-100 km/h, interpolated between the straddling samples.
    target = 100.0 / 3.6
    index = _first_index_where(speed, lambda v: v >= target)
    if index is not None and index > 0:
        previous = speed[index - 1]
        span = speed[index] - previous
        fraction = (target - previous) / span if span > 1e-9 else 0.0
        metrics["zero_to_100kph_s"] = time[index - 1] + fraction * (time[index] - time[index - 1])
    else:
        metrics["zero_to_100kph_s"] = None

    # Distance travelled along the recorded path.
    x = run.column("position_x_m")
    y = run.column("position_y_m")
    distance = 0.0
    for i in range(1, min(len(x), len(y))):
        distance += math.hypot(x[i] - x[i - 1], y[i] - y[i - 1])
    metrics["path_length_m"] = distance

    # Steady-state cornering radius, taken over the last quarter of the run where a
    # constant-radius maneuver has settled: r = v / yaw_rate.
    yaw = run.column("yaw_rate_rad_s")
    tail_start = int(len(run) * 0.75)
    radii = [
        speed[i] / abs(yaw[i])
        for i in range(tail_start, len(run))
        if abs(yaw[i]) > 0.05 and speed[i] > 1.0
    ]
    metrics["steady_radius_m"] = (sum(radii) / len(radii)) if radii else None

    # Transition duration: time between the first and last steering sign change.
    steering = run.column("steering_angle_rad")
    reversals = [
        i
        for i in range(1, len(steering))
        if (steering[i - 1] > 0.05 and steering[i] < -0.05)
        or (steering[i - 1] < -0.05 and steering[i] > 0.05)
    ]
    if len(reversals) >= 2:
        metrics["transition_duration_s"] = time[reversals[-1]] - time[reversals[0]]
    else:
        metrics["transition_duration_s"] = None

    # Recovery time: from peak sideslip back under a tenth of a radian, and staying there.
    sideslip = run.column("body_sideslip_rad")
    if sideslip:
        peak_index = max(range(len(sideslip)), key=lambda i: abs(sideslip[i]))
        recovered = None
        for i in range(peak_index, len(sideslip)):
            if abs(sideslip[i]) < 0.10:
                recovered = i
                break
        metrics["recovery_time_s"] = (
            (time[recovered] - time[peak_index]) if recovered is not None else None
        )
    else:
        metrics["recovery_time_s"] = None

    # Phase 3: load transfer, the acceleration filter, and the separated resistances.
    # None (not zero) when the run predates those columns, so a comparison against an old
    # baseline reports "metric appeared" rather than a fake regression from zero.
    if run.has("load_transfer_n"):
        metrics["peak_load_transfer_n"] = _peak(run.column("load_transfer_n"))
        metrics["peak_filtered_accel_mps2"] = _peak(run.column("filtered_long_accel_mps2"))
        metrics["peak_solved_accel_mps2"] = _peak(run.column("solved_long_accel_mps2"))
        metrics["peak_aero_drag_n"] = _peak(run.column("aero_drag_n"))
        metrics["peak_rolling_resistance_n"] = _peak(run.column("rolling_resistance_n"))
        metrics["min_front_load_n"] = min(run.column("dynamic_front_load_n"))
        metrics["min_rear_load_n"] = min(run.column("dynamic_rear_load_n"))
    else:
        for name in (
            "peak_load_transfer_n",
            "peak_filtered_accel_mps2",
            "peak_solved_accel_mps2",
            "peak_aero_drag_n",
            "peak_rolling_resistance_n",
            "min_front_load_n",
            "min_rear_load_n",
        ):
            metrics[name] = None

    metrics["final_checksum"] = run.value("state_checksum", len(run) - 1) if len(run) else 0.0
    metrics["backlog_drops"] = run.value("backlog_drops", len(run) - 1) if len(run) else 0.0
    return metrics


def gear_sequence(run: Run) -> List[int]:
    """Gear changes as an ordered list — compared exactly, never with a tolerance."""
    gears = run.column("selected_gear")
    sequence: List[int] = []
    for gear in gears:
        value = int(gear)
        if not sequence or sequence[-1] != value:
            sequence.append(value)
    return sequence


def lock_events(run: Run) -> List[Tuple[int, str, int]]:
    """(tick, axle, state) for every wheel-lock transition. Boolean, so also compared exactly."""
    events: List[Tuple[int, str, int]] = []
    for axle in ("front_locked", "rear_locked"):
        previous = 0
        for index, value in enumerate(run.column(axle)):
            state = int(value)
            if state != previous:
                events.append((int(run.value("tick", index)), axle, state))
                previous = state
    return events


# ------------------------------------------------------------------------ comparison --

# Absolute and relative tolerances per column. The keys are matched as suffixes, longest
# first, so "front_normal_load_n" picks up the "_n" force tolerance without being listed.
DEFAULT_TOLERANCES: Dict[str, Tuple[float, float]] = {
    "position_x_m": (0.05, 0.002),
    "position_y_m": (0.05, 0.002),
    "heading_rad": (0.01, 0.002),
    "velocity_longitudinal_mps": (0.05, 0.002),
    "velocity_lateral_mps": (0.05, 0.002),
    "speed_mps": (0.05, 0.002),
    "yaw_rate_rad_s": (0.01, 0.002),
    "steering_angle_rad": (0.005, 0.002),
    "engine_rpm": (25.0, 0.005),
    "body_sideslip_rad": (0.01, 0.005),
    "low_speed_blend": (0.01, 0.005),
    "_slip_angle_rad": (0.005, 0.005),
    "_slip_ratio": (0.01, 0.005),
    "_wheel_omega_rad_s": (0.5, 0.005),
    "_normal_load_n": (25.0, 0.005),
    "_friction_usage": (0.01, 0.005),
    "_torque_nm": (25.0, 0.005),
    # Phase 3 columns.
    "_load_n": (25.0, 0.005),
    "load_transfer_n": (25.0, 0.005),
    "_accel_mps2": (0.05, 0.005),
    "aero_drag_n": (5.0, 0.005),
    "rolling_resistance_n": (5.0, 0.005),
    # Scripted inputs are a pure function of the tick index; they should not drift at all,
    # but they are floats, so give them a hair of absolute slack rather than exactness.
    "_input": (1e-6, 0.0),
    "_n": (25.0, 0.005),
}

EXACT_COLUMNS = ("tick", "selected_gear", "front_locked", "rear_locked")

# Derived metrics are compared as a fraction of the baseline, with an absolute floor so a
# near-zero baseline cannot produce an infinite percentage.
DEFAULT_METRIC_TOLERANCES: Dict[str, Tuple[float, float]] = {
    "zero_to_100kph_s": (0.05, 0.02),
    "peak_speed_mps": (0.10, 0.02),
    "peak_sideslip_rad": (0.02, 0.05),
    "peak_yaw_rate_rad_s": (0.02, 0.05),
    "peak_front_slip_rad": (0.01, 0.05),
    "peak_rear_slip_rad": (0.01, 0.05),
    "peak_friction_usage": (0.01, 0.02),
    "path_length_m": (0.50, 0.02),
    "steady_radius_m": (0.25, 0.05),
    "transition_duration_s": (0.05, 0.05),
    "recovery_time_s": (0.10, 0.10),
    "duration_s": (0.0, 0.0),
    "ticks": (0.0, 0.0),
    # Phase 3 derived metrics.
    "peak_load_transfer_n": (25.0, 0.02),
    "peak_filtered_accel_mps2": (0.05, 0.02),
    "peak_solved_accel_mps2": (0.05, 0.02),
    "peak_aero_drag_n": (5.0, 0.02),
    "peak_rolling_resistance_n": (5.0, 0.02),
    "min_front_load_n": (25.0, 0.02),
    "min_rear_load_n": (25.0, 0.02),
}


def tolerance_for(column: str, overrides: Optional[Dict[str, Tuple[float, float]]] = None):
    table = dict(DEFAULT_TOLERANCES)
    if overrides:
        table.update(overrides)
    if column in table:
        return table[column]
    best: Optional[Tuple[float, float]] = None
    best_length = -1
    for key, value in table.items():
        if column.endswith(key) and len(key) > best_length:
            best, best_length = value, len(key)
    return best if best is not None else (1e-6, 1e-6)


class Difference:
    def __init__(self, kind: str, name: str, baseline, current, delta, tolerance, index=None):
        self.kind = kind  # "exact", "column", "metric"
        self.name = name
        self.baseline = baseline
        self.current = current
        self.delta = delta
        self.tolerance = tolerance
        self.index = index

    @property
    def breached(self) -> bool:
        if self.kind == "exact":
            return self.baseline != self.current
        if self.delta is None or self.tolerance is None:
            return False
        return self.delta > self.tolerance

    def describe(self) -> str:
        if self.kind == "exact":
            return "%s: baseline %s, current %s" % (self.name, self.baseline, self.current)
        return "%s: |delta| %.6g vs tolerance %.6g%s" % (
            self.name,
            self.delta if self.delta is not None else float("nan"),
            self.tolerance if self.tolerance is not None else float("nan"),
            "" if self.index is None else " (first at row %d)" % self.index,
        )


def compare_columns(
    baseline: Run, current: Run, overrides: Optional[Dict[str, Tuple[float, float]]] = None
) -> List[Difference]:
    """Per-column worst-case deviation, tolerance-aware.

    Formatting changes and last-bit float noise must not fail a run; a real change in the
    model must. That is the whole reason this is not `diff`.
    """
    results: List[Difference] = []
    rows = min(len(baseline), len(current))

    for name in baseline.columns:
        if name in EXACT_COLUMNS or name == "state_checksum":
            continue
        if not current.has(name):
            results.append(
                Difference("exact", name + " (missing column)", "present", "absent", None, None)
            )
            continue

        atol, rtol = tolerance_for(name, overrides)

        # Report the row that comes closest to breaching, not the largest raw delta: a big
        # deviation on a big value can be well inside tolerance while a small one is not.
        worst_margin = float("-inf")
        worst_delta = 0.0
        worst_allowed = atol
        worst_index = None
        for i in range(rows):
            a = baseline.value(name, i)
            b = current.value(name, i)
            if math.isnan(a) or math.isnan(b):
                continue
            delta = abs(a - b)
            allowed = atol + rtol * abs(a)
            margin = delta - allowed
            if margin > worst_margin:
                worst_margin, worst_delta, worst_allowed, worst_index = (
                    delta - allowed,
                    delta,
                    allowed,
                    i,
                )
        if worst_index is None:
            worst_delta, worst_allowed = 0.0, atol
        results.append(
            Difference("column", name, None, None, worst_delta, worst_allowed, worst_index)
        )

    return results


def compare_exact(baseline: Run, current: Run) -> List[Difference]:
    results: List[Difference] = []
    results.append(Difference("exact", "tick count", len(baseline), len(current), None, None))
    results.append(
        Difference(
            "exact", "gear sequence", gear_sequence(baseline), gear_sequence(current), None, None
        )
    )
    results.append(
        Difference(
            "exact",
            "wheel lock events",
            len(lock_events(baseline)),
            len(lock_events(current)),
            None,
            None,
        )
    )
    return results


def compare_metrics(
    baseline: Run, current: Run, overrides: Optional[Dict[str, Tuple[float, float]]] = None
) -> List[Difference]:
    table = dict(DEFAULT_METRIC_TOLERANCES)
    if overrides:
        table.update(overrides)

    base_metrics = derived_metrics(baseline)
    current_metrics = derived_metrics(current)
    results: List[Difference] = []

    for name, base_value in base_metrics.items():
        if name in ("final_checksum", "backlog_drops"):
            continue
        current_value = current_metrics.get(name)
        if base_value is None or current_value is None:
            if base_value != current_value:
                results.append(Difference("exact", name, base_value, current_value, None, None))
            continue
        atol, rtol = table.get(name, (0.0, 0.05))
        delta = abs(current_value - base_value)
        allowed = atol + rtol * abs(base_value)
        difference = Difference("metric", name, base_value, current_value, delta, allowed)
        results.append(difference)
    return results


def checksum_matches(baseline: Run, current: Run) -> bool:
    """Only meaningful when both runs came from the same toolchain; the caller decides."""
    if not len(baseline) or not len(current):
        return False
    return baseline.value("state_checksum", len(baseline) - 1) == current.value(
        "state_checksum", len(current) - 1
    )


# ------------------------------------------------------------------------------ plots --


def _nice_bounds(values: Sequence[float]) -> Tuple[float, float]:
    finite = [v for v in values if not math.isnan(v) and not math.isinf(v)]
    if not finite:
        return (0.0, 1.0)
    low, high = min(finite), max(finite)
    if high - low < 1e-9:
        low -= 0.5
        high += 0.5
    padding = (high - low) * 0.08
    return (low - padding, high + padding)


def _escape(text: str) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


SERIES_COLORS = ["#59b8ff", "#ff9f4a", "#8ce99a", "#f28b82", "#c39bff", "#ffd166"]


def line_chart(
    title: str,
    x: Sequence[float],
    series: Sequence[Tuple[str, Sequence[float]]],
    x_label: str = "time (s)",
    y_label: str = "",
    width: int = 640,
    height: int = 260,
    equal_aspect: bool = False,
) -> str:
    """A dependency-free SVG line chart. Plain elements only, so it survives being inlined."""
    left, right, top, bottom = 58, 16, 30, 38
    plot_w = width - left - right
    plot_h = height - top - bottom

    all_y: List[float] = []
    for _, values in series:
        all_y.extend(values)
    y_low, y_high = _nice_bounds(all_y)
    x_low, x_high = _nice_bounds(x)

    if equal_aspect:
        # Keep one metre the same length on both axes, so a trajectory looks like the path
        # the car actually took rather than a stretched version of it.
        x_span = x_high - x_low
        y_span = y_high - y_low
        if x_span / max(plot_w, 1) > y_span / max(plot_h, 1):
            wanted = x_span * plot_h / plot_w
            centre = 0.5 * (y_high + y_low)
            y_low, y_high = centre - wanted / 2, centre + wanted / 2
        else:
            wanted = y_span * plot_w / plot_h
            centre = 0.5 * (x_high + x_low)
            x_low, x_high = centre - wanted / 2, centre + wanted / 2

    def sx(value: float) -> float:
        return left + (value - x_low) / (x_high - x_low) * plot_w

    def sy(value: float) -> float:
        return top + plot_h - (value - y_low) / (y_high - y_low) * plot_h

    parts: List[str] = []
    parts.append(
        '<svg class="chart" viewBox="0 0 %d %d" width="100%%" xmlns="http://www.w3.org/2000/svg" '
        'role="img" aria-label="%s">' % (width, height, _escape(title))
    )
    parts.append('<rect x="0" y="0" width="%d" height="%d" fill="none"/>' % (width, height))
    parts.append('<text x="%d" y="18" class="chart-title">%s</text>' % (left, _escape(title)))

    for step in range(5):
        value = y_low + (y_high - y_low) * step / 4.0
        y = sy(value)
        parts.append(
            '<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" class="grid"/>'
            % (left, y, left + plot_w, y)
        )
        parts.append(
            '<text x="%.1f" y="%.1f" class="tick" text-anchor="end">%.3g</text>'
            % (left - 6, y + 3, value)
        )
    for step in range(5):
        value = x_low + (x_high - x_low) * step / 4.0
        x_pixel = sx(value)
        parts.append(
            '<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" class="grid"/>'
            % (x_pixel, top, x_pixel, top + plot_h)
        )
        parts.append(
            '<text x="%.1f" y="%.1f" class="tick" text-anchor="middle">%.3g</text>'
            % (x_pixel, top + plot_h + 14, value)
        )

    parts.append(
        '<text x="%.1f" y="%.1f" class="axis" text-anchor="middle">%s</text>'
        % (left + plot_w / 2, height - 6, _escape(x_label))
    )
    if y_label:
        parts.append(
            '<text transform="translate(12,%.1f) rotate(-90)" class="axis" '
            'text-anchor="middle">%s</text>' % (top + plot_h / 2, _escape(y_label))
        )

    for index, (name, values) in enumerate(series):
        color = SERIES_COLORS[index % len(SERIES_COLORS)]
        points = []
        for i in range(min(len(x), len(values))):
            if math.isnan(values[i]):
                continue
            points.append("%.2f,%.2f" % (sx(x[i]), sy(values[i])))
        if points:
            parts.append(
                '<polyline fill="none" stroke="%s" stroke-width="1.4" points="%s"/>'
                % (color, " ".join(points))
            )
        parts.append(
            '<rect x="%.1f" y="%.1f" width="9" height="9" fill="%s"/>'
            % (left + plot_w - 120, top + 4 + index * 14, color)
        )
        parts.append(
            '<text x="%.1f" y="%.1f" class="legend">%s</text>'
            % (left + plot_w - 106, top + 12 + index * 14, _escape(name))
        )

    parts.append("</svg>")
    return "".join(parts)
