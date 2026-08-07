#!/usr/bin/env python3
"""Learn a checkpoint-valid racing line with a deterministic neural policy search.

This is an offline ML/RL-style test, not a replacement for the C vehicle simulator. A small
MLP maps local track features to a lateral offset from the authored centreline. A deterministic
cross-entropy neuroevolution loop treats each candidate as an episode policy and minimizes a
friction-limited lap-time model. The resulting line is then consumed by the game as a generated
racing-line artifact and checked against every ordered checkpoint.

The objective follows autonomous-racing RL literature: maximize forward progress (equivalently,
minimize lap time), keep the line within the surface, and respect a friction-limited speed
profile. The learned line is deliberately separated from the centreline so the test can prove
that the target is an optimized path rather than merely the authored geometry.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

G = 9.81
MAX_SPEED = 45.0
# The driver only ever spends corneringGripFraction (0.75) of the tyre's lateral grip, so
# planning the line at the tyre's full mu produces apexes no car on the roster can carry speed
# through. Lap times scale together across the roster, so one representative level is enough.
LAT_GRIP = 0.78 * G
ACCEL = 4.0
BRAKE = 8.0
# How much asphalt the line leaves between itself and the edge: half a car plus the tracking
# error a pure-pursuit driver shows on a curved line. Without it the optimiser puts the line
# where a slightly understeering car has no road left, and that car ends up slower, not faster.
LINE_EDGE_MARGIN_M = 2.4
HIDDEN = 12
CONTROL_POINTS = 48
SMOOTH_WINDOW = 9


@dataclass(frozen=True)
class TrackModel:
    centers: np.ndarray
    half_widths: np.ndarray
    checkpoints: np.ndarray
    checkpoint_forward: np.ndarray
    checkpoint_half_width: float
    track_id: str


def make_chicane() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    spacing = 4.0
    radius = 45.0
    half_x = 100.0
    far_y = 90.0
    chicane_offset = 16.0
    chicane_span = 80.0
    entry_x = 55.0
    nodes: list[tuple[float, float]] = []
    widths: list[float] = []

    for x in np.arange(-half_x, half_x - 0.5 * spacing, spacing):
        nodes.append((float(x), 0.0))
        widths.append(8.0)
    steps = int((math.pi * radius) / spacing)
    for i in range(steps):
        theta = -0.5 * math.pi + math.pi * (i / steps)
        nodes.append((half_x + radius * math.cos(theta), radius + radius * math.sin(theta)))
        widths.append(8.0)
    for x in np.arange(half_x, -half_x + 0.5 * spacing, -spacing):
        into = entry_x - float(x)
        in_chicane = 0.0 <= into <= chicane_span
        y = far_y
        width = 8.0
        if in_chicane:
            u = into / chicane_span
            y += chicane_offset * math.sin(math.pi * u) ** 2
            width = 6.0
        nodes.append((float(x), y))
        widths.append(width)
    for i in range(steps):
        theta = 0.5 * math.pi + math.pi * (i / steps)
        nodes.append((-half_x + radius * math.cos(theta), radius + radius * math.sin(theta)))
        widths.append(8.0)

    checkpoints = np.array(
        [
            (-60.0, 0.0),
            (half_x, 0.0),
            (half_x + radius, radius),
            (half_x, far_y),
            (entry_x, far_y),
            (entry_x - 0.5 * chicane_span, far_y + chicane_offset),
            (entry_x - chicane_span, far_y),
            (-half_x - radius, radius),
        ],
        dtype=np.float64,
    )
    forwards = np.array(
        [
            (1.0, 0.0),
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (-1.0, 0.0),
            (-1.0, 0.0),
            (-1.0, 0.0),
            (0.0, -1.0),
        ],
        dtype=np.float64,
    )
    return (
        np.array(nodes, dtype=np.float64),
        np.array(widths, dtype=np.float64),
        checkpoints,
        forwards,
    )


def make_track(track_id: str) -> TrackModel:
    centers, widths, checkpoints, forwards = make_chicane()
    if track_id == "chicane":
        return TrackModel(centers, widths, checkpoints, forwards, 10.0, track_id)

    if track_id == "sprint":
        transform = np.array([[0.82, 0.0], [0.0, 0.88]])
        centers = centers @ transform.T
        checkpoints = checkpoints @ transform.T
        forwards = forwards @ transform.T
        forwards /= np.linalg.norm(forwards, axis=1, keepdims=True)
        return TrackModel(centers, widths, checkpoints, forwards, 10.0, track_id)

    if track_id == "technical":
        scale_x, scale_y, skew_x, skew_y = 0.62, 0.58, 0.08, 0.05
        transform = np.array([[scale_x, skew_x], [skew_y, scale_y]])
        centers = centers @ transform.T
        checkpoints = checkpoints @ transform.T
        forwards = forwards @ transform.T
        forwards /= np.linalg.norm(forwards, axis=1, keepdims=True)
        widths = np.where(widths <= 6.0, 4.5, 5.8)
        return TrackModel(centers, widths, checkpoints, forwards, 10.0, track_id)

    raise ValueError(f"unsupported track: {track_id}")


def nearest_checkpoint_indices(track: TrackModel) -> np.ndarray:
    return np.array(
        [np.argmin(np.linalg.norm(track.centers - point, axis=1)) for point in track.checkpoints],
        dtype=np.int32,
    )


def signed_curvature(points: np.ndarray) -> np.ndarray:
    prev_points = np.roll(points, 1, axis=0)
    next_points = np.roll(points, -1, axis=0)
    ab = points - prev_points
    bc = next_points - points
    ca = prev_points - next_points
    lengths = np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ca, axis=1)
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    return 2.0 * cross / np.maximum(lengths, 1e-9)


def geometry_features(track: TrackModel) -> np.ndarray:
    """Local shape as the policy sees it.

    Curvature is SIGNED so the policy can tell a left-hander from a right-hander and place the
    line on the correct side; look-behind and look-ahead samples let it turn in before an apex
    and unwind after it, which is what separates a racing line from a constant inward offset.
    """
    kappa = signed_curvature(track.centers)
    scaled = np.clip(kappa * 30.0, -1.0, 1.0)
    phase = np.arange(len(track.centers), dtype=np.float64) / len(track.centers)
    return np.column_stack(
        (
            scaled,
            np.roll(scaled, -6),
            np.roll(scaled, -14),
            np.roll(scaled, -26),
            np.roll(scaled, 6),
            np.roll(scaled, 14),
            np.abs(scaled),
            track.half_widths / 8.0,
            np.sin(2.0 * math.pi * phase),
            np.cos(2.0 * math.pi * phase),
        )
    )


FEATURE_COUNT = 10


def unpack(theta: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    a = FEATURE_COUNT * HIDDEN
    b = a + HIDDEN
    c = b + HIDDEN
    w1 = theta[:a].reshape(FEATURE_COUNT, HIDDEN)
    b1 = theta[a:b]
    w2 = theta[b:c].reshape(HIDDEN, 1)
    b2 = theta[c : c + 1]
    return w1, b1, w2, b2


def circular_smooth(values: np.ndarray, window: int) -> np.ndarray:
    """Hann-weighted circular moving average.

    The authored circuits have piecewise-constant curvature — straight, then arc — so any policy
    reading curvature emits step changes at every curve entry. Smoothing the offset profile makes
    the parameterisation produce inherently drivable lines instead of forcing the optimiser to
    buy smoothness back through a penalty, which it can only do by shrinking the whole line.
    """
    weights = np.hanning(window + 2)[1:-1]
    weights /= weights.sum()
    padded = np.r_[values[-window:], values, values[:window]]
    return np.convolve(padded, weights, mode="same")[window : window + len(values)]


def policy_offsets(theta: np.ndarray, features: np.ndarray, half_widths: np.ndarray) -> np.ndarray:
    w1, b1, w2, b2 = unpack(theta)
    hidden = np.tanh(features @ w1 + b1)
    raw = np.tanh((hidden @ w2 + b2)[:, 0])
    amplitude = np.maximum(half_widths - LINE_EDGE_MARGIN_M, 0.0)
    return circular_smooth(raw * amplitude, SMOOTH_WINDOW)


def polyline_curvature(line: np.ndarray) -> np.ndarray:
    """Menger curvature at each vertex of a closed polyline, 1/m."""
    prev_line = np.roll(line, 1, axis=0)
    next_line = np.roll(line, -1, axis=0)
    ab = line - prev_line
    bc = next_line - line
    ca = prev_line - next_line
    denom = np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ca, axis=1)
    return 2.0 * np.abs(ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]) / np.maximum(denom, 1e-9)


def lap_time_model_s(ds: np.ndarray, curvature: np.ndarray) -> float:
    """Forward/backward speed-profile pass: corner limit, then what can be reached and stopped."""
    speeds = np.minimum(MAX_SPEED, np.sqrt(LAT_GRIP / np.maximum(curvature, 1e-5)))
    count = len(ds)
    for _ in range(5):
        for i in range(count):
            j = (i + 1) % count
            speeds[j] = min(speeds[j], math.sqrt(max(0.0, speeds[i] ** 2 + 2.0 * ACCEL * ds[i])))
        for i in range(count - 1, -1, -1):
            j = (i + 1) % count
            speeds[i] = min(speeds[i], math.sqrt(max(0.0, speeds[j] ** 2 + 2.0 * BRAKE * ds[i])))
    return float(np.sum(ds / np.maximum((speeds + np.roll(speeds, -1)) * 0.5, 1.0)))


def line_metrics(
    track: TrackModel, offsets: np.ndarray, curvature_budget: float | None = None
) -> tuple[float, float, float, np.ndarray]:
    points = track.centers.copy()
    tangents = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    tangents /= np.maximum(np.linalg.norm(tangents, axis=1, keepdims=True), 1e-9)
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    line = points + normals * offsets[:, None]

    ds = np.linalg.norm(np.roll(line, -1, axis=0) - line, axis=1)
    curvature = polyline_curvature(line)

    lap_time = lap_time_model_s(ds, curvature)

    wrapped = np.r_[offsets, offsets[:2]]
    kink = np.diff(wrapped, 2)[: len(offsets)]
    # Drivability. A lap-time model rewards any kink that shortens the path, but a real
    # pure-pursuit driver lags a step change in lateral offset and runs wide. The quantity that
    # actually defeats the controller is the SECOND difference of the offset profile — a
    # reversal of lateral drift — not the drift itself, so a steady sweep across the track is
    # left unpenalised and only unfollowable kinks are priced.
    drivability = float(600.0 * np.mean(kink**2))

    checkpoint_error = max(abs(float(offsets[i])) for i in nearest_checkpoint_indices(track))
    # The gates are intentionally wider than the racing surface; this is a hard validity check,
    # not a soft preference. The chosen bound should therefore never be paid by valid candidates.
    gate_violation = max(0.0, checkpoint_error - track.checkpoint_half_width)

    # One line serves a roster whose grip differs by a wide margin, and the corner speed a car
    # can carry is sqrt(mu*g/kappa). A line that tightens the tightest corner is therefore only
    # faster for cars with grip to spare; the low-grip car crawls through the new apex and loses
    # more than the shorter path saved. Capping peak curvature at the road's own value makes the
    # gain come from shortening and straightening, which every car can use.
    excess = 0.0
    if curvature_budget is not None:
        excess = max(0.0, float(curvature.max()) - curvature_budget)
    penalty = drivability + 1000.0 * gate_violation * gate_violation + 40000.0 * excess * excess
    return lap_time, penalty, checkpoint_error, line


def line_cost(lap_time: float, penalty: float) -> float:
    return lap_time + penalty


def train(track: TrackModel, seed: int, iterations: int, population: int) -> dict:
    rng = np.random.default_rng(seed)
    features = geometry_features(track)
    parameter_count = FEATURE_COUNT * HIDDEN + HIDDEN + HIDDEN + 1
    mean = np.zeros(parameter_count, dtype=np.float64)
    sigma = np.full(parameter_count, 0.35, dtype=np.float64)

    zero_offsets = np.zeros(len(track.centers), dtype=np.float64)
    baseline_time, _, baseline_gate_error, baseline_line = line_metrics(track, zero_offsets)
    curvature_budget = float(polyline_curvature(baseline_line).max())
    best = (
        line_cost(baseline_time, 0.0),
        np.zeros(parameter_count),
        baseline_time,
        baseline_gate_error,
        baseline_line,
    )
    elite_count = max(6, population // 8)

    for _ in range(iterations):
        candidates = rng.normal(mean, sigma, size=(population, parameter_count))
        candidates[0] = mean
        evaluated = []
        for theta in candidates:
            offsets = policy_offsets(theta, features, track.half_widths)
            lap_time, penalty, gate_error, line = line_metrics(track, offsets, curvature_budget)
            evaluated.append((line_cost(lap_time, penalty), theta, lap_time, gate_error, line))
        evaluated.sort(key=lambda item: item[0])
        if evaluated[0][0] < best[0]:
            best = evaluated[0]
        elites = np.array([item[1] for item in evaluated[:elite_count]])
        mean = 0.75 * mean + 0.25 * np.mean(elites, axis=0)
        sigma = np.maximum(0.03, 0.75 * sigma + 0.25 * np.std(elites, axis=0))

    _, theta, lap_time, gate_error, line = best
    offsets = policy_offsets(theta, features, track.half_widths)
    control_phases = np.linspace(0.0, 1.0, CONTROL_POINTS, endpoint=False)
    control_offsets = np.interp(
        control_phases,
        np.arange(len(offsets) + 1) / len(offsets),
        np.r_[offsets, offsets[0]],
    )
    return {
        "schema_version": "1.0.0",
        "method": "cross_entropy_neuroevolution_mlp",
        "objective": "minimum friction-limited lap time through ordered checkpoints",
        "track": track.track_id,
        "seed": seed,
        "iterations": iterations,
        "population": population,
        "node_count": len(track.centers),
        "control_point_count": CONTROL_POINTS,
        "lat_grip_g": LAT_GRIP / G,
        "edge_margin_m": LINE_EDGE_MARGIN_M,
        "baseline_lap_time_s": baseline_time,
        "optimized_lap_time_s": lap_time,
        "improvement_percent": 100.0 * (baseline_time - lap_time) / baseline_time,
        "max_checkpoint_lateral_error_m": gate_error,
        "checkpoint_count": len(track.checkpoints),
        "checkpoint_valid": gate_error <= track.checkpoint_half_width,
        "control_offsets_m": control_offsets.tolist(),
        "learned_offsets_m": offsets.tolist(),
        "line_points_m": line.tolist(),
        "policy_parameter_count": int(len(theta)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--track", default="technical", choices=("chicane", "sprint", "technical"))
    parser.add_argument("--seed", type=int, default=20260807)
    parser.add_argument("--iterations", type=int, default=120)
    parser.add_argument("--population", type=int, default=96)
    parser.add_argument(
        "--output", type=Path, default=Path("artifacts/racing_line/technical_nn_line.json")
    )
    args = parser.parse_args()

    result = train(make_track(args.track), args.seed, args.iterations, args.population)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                key: result[key]
                for key in (
                    "method",
                    "track",
                    "seed",
                    "baseline_lap_time_s",
                    "optimized_lap_time_s",
                    "improvement_percent",
                    "max_checkpoint_lateral_error_m",
                    "checkpoint_valid",
                    "control_offsets_m",
                    "policy_parameter_count",
                )
            },
            indent=2,
        )
    )
    return (
        0
        if result["checkpoint_valid"]
        and result["optimized_lap_time_s"] < result["baseline_lap_time_s"]
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
