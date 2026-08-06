#!/usr/bin/env python3
"""Measure alpha-mask instability when a pixel-art sprite is rotated with point sampling."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

from compare_car_rgba import Image, PngError, read_png


@dataclass(frozen=True)
class RotationMetrics:
    source_width: int
    source_height: int
    steps: int
    canvas_size: int
    source_occupied_pixels: int
    min_occupied_pixels: int
    max_occupied_pixels: int
    occupied_span_ratio: float
    max_centroid_drift_px: float
    mean_adjacent_xor_ratio: float
    max_adjacent_xor_ratio: float


def source_mask(image: Image, alpha_threshold: int) -> list[bool]:
    return [image.rgba[i + 3] > alpha_threshold for i in range(0, len(image.rgba), 4)]


def default_pivot(mask: list[bool], width: int, height: int) -> tuple[float, float]:
    points = [
        ((index % width) + 0.5, (index // width) + 0.5)
        for index, occupied in enumerate(mask)
        if occupied
    ]
    if not points:
        return width * 0.5, height * 0.5
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def rotate_mask(
    mask: list[bool],
    src_width: int,
    src_height: int,
    pivot_x: float,
    pivot_y: float,
    angle_rad: float,
    canvas_size: int,
) -> list[bool]:
    destination = [False] * (canvas_size * canvas_size)
    canvas_pivot = canvas_size * 0.5
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)

    for y in range(canvas_size):
        dy = (y + 0.5) - canvas_pivot
        for x in range(canvas_size):
            dx = (x + 0.5) - canvas_pivot
            # Inverse rotation: destination pixel centre back into source space.
            source_x = pivot_x + dx * cosine + dy * sine
            source_y = pivot_y - dx * sine + dy * cosine
            sample_x = math.floor(source_x)
            sample_y = math.floor(source_y)
            if 0 <= sample_x < src_width and 0 <= sample_y < src_height:
                destination[y * canvas_size + x] = mask[sample_y * src_width + sample_x]
    return destination


def mask_stats(mask: list[bool], width: int) -> tuple[int, float, float]:
    occupied = [index for index, value in enumerate(mask) if value]
    if not occupied:
        return 0, width * 0.5, width * 0.5
    count = len(occupied)
    centroid_x = sum((index % width) + 0.5 for index in occupied) / count
    centroid_y = sum((index // width) + 0.5 for index in occupied) / count
    return count, centroid_x, centroid_y


def xor_ratio(a: list[bool], b: list[bool]) -> float:
    union = 0
    differing = 0
    for av, bv in zip(a, b):
        if av or bv:
            union += 1
        if av != bv:
            differing += 1
    return differing / union if union else 0.0


def measure(
    image: Image,
    steps: int,
    alpha_threshold: int,
    pivot_x: float | None,
    pivot_y: float | None,
) -> RotationMetrics:
    mask = source_mask(image, alpha_threshold)
    source_count = sum(mask)
    default_x, default_y = default_pivot(mask, image.width, image.height)
    px = default_x if pivot_x is None else pivot_x
    py = default_y if pivot_y is None else pivot_y
    canvas_size = math.ceil(math.hypot(image.width, image.height)) + 4
    if canvas_size % 2:
        canvas_size += 1

    counts: list[int] = []
    drifts: list[float] = []
    adjacent: list[float] = []
    previous: list[bool] | None = None
    canvas_pivot = canvas_size * 0.5

    for step in range(steps):
        angle = math.tau * step / steps
        rotated = rotate_mask(mask, image.width, image.height, px, py, angle, canvas_size)
        count, centroid_x, centroid_y = mask_stats(rotated, canvas_size)
        counts.append(count)
        drifts.append(math.hypot(centroid_x - canvas_pivot, centroid_y - canvas_pivot))
        if previous is not None:
            adjacent.append(xor_ratio(previous, rotated))
        previous = rotated

    # Close the cycle so the last-to-first transition is measured too.
    first = rotate_mask(mask, image.width, image.height, px, py, 0.0, canvas_size)
    if previous is not None:
        adjacent.append(xor_ratio(previous, first))

    min_count = min(counts, default=0)
    max_count = max(counts, default=0)
    span_ratio = (max_count - min_count) / source_count if source_count else 0.0
    return RotationMetrics(
        source_width=image.width,
        source_height=image.height,
        steps=steps,
        canvas_size=canvas_size,
        source_occupied_pixels=source_count,
        min_occupied_pixels=min_count,
        max_occupied_pixels=max_count,
        occupied_span_ratio=span_ratio,
        max_centroid_drift_px=max(drifts, default=0.0),
        mean_adjacent_xor_ratio=(sum(adjacent) / len(adjacent)) if adjacent else 0.0,
        max_adjacent_xor_ratio=max(adjacent, default=0.0),
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure point-sampled rotation shimmer in a rendered car sprite PNG."
    )
    parser.add_argument("sprite", type=Path)
    parser.add_argument("--steps", type=int, default=64)
    parser.add_argument("--alpha-threshold", type=int, default=0)
    parser.add_argument("--pivot-x", type=float)
    parser.add_argument("--pivot-y", type=float)
    parser.add_argument("--max-occupied-span-ratio", type=float, default=1.0)
    parser.add_argument("--max-centroid-drift-px", type=float, default=1e9)
    parser.add_argument("--max-adjacent-xor-ratio", type=float, default=1.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    if args.steps < 4:
        parser.error("--steps must be at least 4")
    if not 0 <= args.alpha_threshold <= 254:
        parser.error("--alpha-threshold must be in [0, 254]")
    if (args.pivot_x is None) != (args.pivot_y is None):
        parser.error("--pivot-x and --pivot-y must be supplied together")
    if args.max_occupied_span_ratio < 0:
        parser.error("--max-occupied-span-ratio must be non-negative")
    if args.max_centroid_drift_px < 0:
        parser.error("--max-centroid-drift-px must be non-negative")
    if not 0 <= args.max_adjacent_xor_ratio <= 1:
        parser.error("--max-adjacent-xor-ratio must be in [0, 1]")
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        image = read_png(args.sprite)
        metrics = measure(
            image,
            args.steps,
            args.alpha_threshold,
            args.pivot_x,
            args.pivot_y,
        )
    except (OSError, PngError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    failures: list[str] = []
    if metrics.occupied_span_ratio > args.max_occupied_span_ratio:
        failures.append(
            f"occupied span ratio {metrics.occupied_span_ratio:.6f} exceeds "
            f"{args.max_occupied_span_ratio:.6f}"
        )
    if metrics.max_centroid_drift_px > args.max_centroid_drift_px:
        failures.append(
            f"centroid drift {metrics.max_centroid_drift_px:.6f}px exceeds "
            f"{args.max_centroid_drift_px:.6f}px"
        )
    if metrics.max_adjacent_xor_ratio > args.max_adjacent_xor_ratio:
        failures.append(
            f"adjacent XOR ratio {metrics.max_adjacent_xor_ratio:.6f} exceeds "
            f"{args.max_adjacent_xor_ratio:.6f}"
        )

    payload = {**asdict(metrics), "passed": not failures, "failures": failures}
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(
            f"{metrics.source_width}x{metrics.source_height}, steps={metrics.steps}: "
            f"area={metrics.min_occupied_pixels}..{metrics.max_occupied_pixels} "
            f"(span={metrics.occupied_span_ratio:.6%}), "
            f"centroid_drift={metrics.max_centroid_drift_px:.4f}px, "
            f"adjacent_xor_mean={metrics.mean_adjacent_xor_ratio:.6%}, "
            f"adjacent_xor_max={metrics.max_adjacent_xor_ratio:.6%}"
        )
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
