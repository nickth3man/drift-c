#!/usr/bin/env python3
"""Compare two deterministic car-sprite PNG artifacts without third-party packages."""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import zlib
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


class PngError(ValueError):
    pass


@dataclass(frozen=True)
class Image:
    width: int
    height: int
    rgba: bytes


@dataclass(frozen=True)
class Metrics:
    width: int
    height: int
    pixels: int
    differing_pixels: int
    differing_ratio: float
    alpha_union_pixels: int
    alpha_xor_pixels: int
    alpha_xor_ratio: float
    rgb_rmse: float
    rgba_rmse: float
    max_channel_delta: int


def _paeth(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa = abs(p - a)
    pb = abs(p - b)
    pc = abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def _unfilter_row(filter_type: int, row: bytearray, prev: bytes, bpp: int) -> bytes:
    for i in range(len(row)):
        left = row[i - bpp] if i >= bpp else 0
        up = prev[i] if prev else 0
        up_left = prev[i - bpp] if prev and i >= bpp else 0
        if filter_type == 0:
            value = row[i]
        elif filter_type == 1:
            value = row[i] + left
        elif filter_type == 2:
            value = row[i] + up
        elif filter_type == 3:
            value = row[i] + ((left + up) // 2)
        elif filter_type == 4:
            value = row[i] + _paeth(left, up, up_left)
        else:
            raise PngError(f"unsupported PNG filter type: {filter_type}")
        row[i] = value & 0xFF
    return bytes(row)


def read_png(path: Path) -> Image:
    data = path.read_bytes()
    if not data.startswith(PNG_SIGNATURE):
        raise PngError(f"{path}: not a PNG file")

    offset = len(PNG_SIGNATURE)
    width = height = bit_depth = color_type = interlace = None
    compressed = bytearray()

    while offset + 12 <= len(data):
        length = struct.unpack_from(">I", data, offset)[0]
        chunk_type = data[offset + 4 : offset + 8]
        chunk_start = offset + 8
        chunk_end = chunk_start + length
        crc_end = chunk_end + 4
        if crc_end > len(data):
            raise PngError(f"{path}: truncated PNG chunk")
        chunk = data[chunk_start:chunk_end]
        expected_crc = struct.unpack_from(">I", data, chunk_end)[0]
        actual_crc = zlib.crc32(chunk_type)
        actual_crc = zlib.crc32(chunk, actual_crc) & 0xFFFFFFFF
        if actual_crc != expected_crc:
            raise PngError(f"{path}: CRC mismatch in {chunk_type.decode('ascii', 'replace')}")

        if chunk_type == b"IHDR":
            if length != 13:
                raise PngError(f"{path}: invalid IHDR length")
            width, height, bit_depth, color_type, compression, filtering, interlace = struct.unpack(
                ">IIBBBBB", chunk
            )
            if width <= 0 or height <= 0:
                raise PngError(f"{path}: invalid image dimensions")
            if compression != 0 or filtering != 0:
                raise PngError(f"{path}: unsupported PNG compression/filter method")
            if interlace != 0:
                raise PngError(f"{path}: interlaced PNGs are not supported")
            if bit_depth != 8 or color_type not in (2, 6):
                raise PngError(
                    f"{path}: only 8-bit RGB and RGBA PNGs are supported "
                    f"(got depth={bit_depth}, type={color_type})"
                )
        elif chunk_type == b"IDAT":
            compressed.extend(chunk)
        elif chunk_type == b"IEND":
            break
        offset = crc_end

    if None in (width, height, bit_depth, color_type, interlace):
        raise PngError(f"{path}: missing IHDR")
    if not compressed:
        raise PngError(f"{path}: missing IDAT data")

    channels = 4 if color_type == 6 else 3
    stride = width * channels
    expected = height * (stride + 1)
    try:
        raw = zlib.decompress(bytes(compressed))
    except zlib.error as exc:
        raise PngError(f"{path}: invalid compressed pixel data: {exc}") from exc
    if len(raw) != expected:
        raise PngError(f"{path}: decoded {len(raw)} bytes, expected {expected}")

    rows: list[bytes] = []
    cursor = 0
    prev = b""
    for _ in range(height):
        filter_type = raw[cursor]
        cursor += 1
        row = bytearray(raw[cursor : cursor + stride])
        cursor += stride
        decoded = _unfilter_row(filter_type, row, prev, channels)
        rows.append(decoded)
        prev = decoded

    if color_type == 6:
        rgba = b"".join(rows)
    else:
        expanded = bytearray(width * height * 4)
        dst = 0
        for row in rows:
            for src in range(0, len(row), 3):
                expanded[dst : dst + 3] = row[src : src + 3]
                expanded[dst + 3] = 255
                dst += 4
        rgba = bytes(expanded)

    return Image(width=width, height=height, rgba=rgba)


def compare(a: Image, b: Image) -> Metrics:
    if (a.width, a.height) != (b.width, b.height):
        raise ValueError(
            f"image dimensions differ: {a.width}x{a.height} vs {b.width}x{b.height}"
        )

    pixels = a.width * a.height
    differing = 0
    alpha_union = 0
    alpha_xor = 0
    rgb_sq = 0
    rgba_sq = 0
    max_delta = 0

    for i in range(0, len(a.rgba), 4):
        ap = a.rgba[i : i + 4]
        bp = b.rgba[i : i + 4]
        if ap != bp:
            differing += 1
        a_opaque = ap[3] != 0
        b_opaque = bp[3] != 0
        if a_opaque or b_opaque:
            alpha_union += 1
        if a_opaque != b_opaque:
            alpha_xor += 1
        for channel in range(4):
            delta = abs(ap[channel] - bp[channel])
            max_delta = max(max_delta, delta)
            rgba_sq += delta * delta
            if channel < 3:
                rgb_sq += delta * delta

    return Metrics(
        width=a.width,
        height=a.height,
        pixels=pixels,
        differing_pixels=differing,
        differing_ratio=(differing / pixels) if pixels else 0.0,
        alpha_union_pixels=alpha_union,
        alpha_xor_pixels=alpha_xor,
        alpha_xor_ratio=(alpha_xor / alpha_union) if alpha_union else 0.0,
        rgb_rmse=math.sqrt(rgb_sq / (pixels * 3)) if pixels else 0.0,
        rgba_rmse=math.sqrt(rgba_sq / (pixels * 4)) if pixels else 0.0,
        max_channel_delta=max_delta,
    )


def _failures(metrics: Metrics, args: argparse.Namespace) -> Iterable[str]:
    if metrics.differing_ratio > args.max_differing_ratio:
        yield (
            f"differing ratio {metrics.differing_ratio:.8f} exceeds "
            f"{args.max_differing_ratio:.8f}"
        )
    if metrics.alpha_xor_ratio > args.max_alpha_xor_ratio:
        yield (
            f"alpha XOR ratio {metrics.alpha_xor_ratio:.8f} exceeds "
            f"{args.max_alpha_xor_ratio:.8f}"
        )
    if metrics.rgb_rmse > args.max_rgb_rmse:
        yield f"RGB RMSE {metrics.rgb_rmse:.6f} exceeds {args.max_rgb_rmse:.6f}"
    if metrics.rgba_rmse > args.max_rgba_rmse:
        yield f"RGBA RMSE {metrics.rgba_rmse:.6f} exceeds {args.max_rgba_rmse:.6f}"
    if metrics.max_channel_delta > args.max_channel_delta:
        yield (
            f"max channel delta {metrics.max_channel_delta} exceeds "
            f"{args.max_channel_delta}"
        )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare actual RGBA output from two deterministic car-sprite PNG artifacts."
    )
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--max-differing-ratio", type=float, default=0.0)
    parser.add_argument("--max-alpha-xor-ratio", type=float, default=0.0)
    parser.add_argument("--max-rgb-rmse", type=float, default=0.0)
    parser.add_argument("--max-rgba-rmse", type=float, default=0.0)
    parser.add_argument("--max-channel-delta", type=int, default=0)
    parser.add_argument("--json", action="store_true", help="emit machine-readable metrics")
    args = parser.parse_args(argv)

    if not 0.0 <= args.max_differing_ratio <= 1.0:
        parser.error("--max-differing-ratio must be in [0, 1]")
    if not 0.0 <= args.max_alpha_xor_ratio <= 1.0:
        parser.error("--max-alpha-xor-ratio must be in [0, 1]")
    if not (math.isfinite(args.max_rgb_rmse) and args.max_rgb_rmse >= 0.0):
        parser.error("--max-rgb-rmse must be non-negative and finite")
    if not (math.isfinite(args.max_rgba_rmse) and args.max_rgba_rmse >= 0.0):
        parser.error("--max-rgba-rmse must be non-negative and finite")
    if not 0 <= args.max_channel_delta <= 255:
        parser.error("--max-channel-delta must be in [0, 255]")
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        baseline = read_png(args.baseline)
        candidate = read_png(args.candidate)
        metrics = compare(baseline, candidate)
    except (OSError, PngError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    failures = list(_failures(metrics, args))
    payload = {**asdict(metrics), "passed": not failures, "failures": failures}
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(
            f"{metrics.width}x{metrics.height}: "
            f"different={metrics.differing_pixels}/{metrics.pixels} "
            f"({metrics.differing_ratio:.6%}), "
            f"alpha_xor={metrics.alpha_xor_pixels}/{metrics.alpha_union_pixels} "
            f"({metrics.alpha_xor_ratio:.6%}), "
            f"rgb_rmse={metrics.rgb_rmse:.6f}, "
            f"rgba_rmse={metrics.rgba_rmse:.6f}, "
            f"max_delta={metrics.max_channel_delta}"
        )
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
