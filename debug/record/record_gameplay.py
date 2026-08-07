import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

TELEMETRY_DIR = Path("artifacts/telemetry")
TEST_EXE = Path("build/tests/drifty_tests.exe")
WHEEL_RADIUS_M = 0.31

# These names match the telemetry files emitted by the C test runner.  They are
# intentionally explicit: aliases such as "flick" and "offroad" hid the real
# scenarios being measured.
PHASE2_SOURCES = {
    "phase2_determinism": "phase2_determinism.csv",
    "phase2_determinism_repeat": "phase2_determinism_repeat.csv",
    "phase2_launch_stop": "phase2_launch_stop.csv",
    "phase2_coast_down": "phase2_coast_down.csv",
    "phase2_power_oversteer": "phase2_power_oversteer.csv",
    "phase2_handbrake_entry": "phase2_handbrake_entry.csv",
    "phase2_braking_cornering": "phase2_braking_cornering.csv",
}


def discover_scenario_sources():
    sources = dict(PHASE2_SOURCES)
    for csv_path in TELEMETRY_DIR.glob("scenario_*.csv"):
        sources[csv_path.stem] = csv_path.name
    return sources


INT_FIELDS = {
    "tick",
    "selected_gear",
    "front_locked",
    "rear_locked",
    "substep_count",
    "backlog_drops",
    "state_checksum",
    "surface_front_left",
    "surface_front_right",
    "surface_rear_left",
    "surface_rear_right",
}


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def get_next_log_dir():
    base_logs = Path("debug/record/logs").resolve()
    base_logs.mkdir(parents=True, exist_ok=True)
    today_str = time.strftime("%d-%m-%y")

    max_id = 0
    for entry in base_logs.iterdir():
        parts = entry.name.split("_")
        if len(parts) >= 2 and parts[0].isdigit():
            max_id = max(max_id, int(parts[0]))

    target_dir = base_logs / f"{max_id + 1:04d}_{today_str}"
    target_dir.mkdir(parents=True, exist_ok=False)
    return target_dir


def decode_output(data):
    return data.decode("utf-8", errors="replace") if data else ""


def run_headless_simulation():
    """Run the C suite and fail closed if build or execution fails."""
    test_exe = TEST_EXE.resolve()
    build_metadata = None
    if not test_exe.exists():
        build_command = ["cmd.exe", "/c", "build.bat", "--tests"]
        build_started = time.perf_counter()
        build_result = subprocess.run(build_command, capture_output=True, check=False)
        build_metadata = {
            "command": build_command,
            "returncode": build_result.returncode,
            "duration_seconds": round(time.perf_counter() - build_started, 3),
            "stdout": decode_output(build_result.stdout),
            "stderr": decode_output(build_result.stderr),
        }
        if build_result.returncode != 0:
            raise RuntimeError(
                "headless test build failed with exit code "
                f"{build_result.returncode}: {build_metadata['stderr']}"
            )

    command = [str(test_exe)]
    started_at = utc_now()
    started = time.perf_counter()
    result = subprocess.run(command, capture_output=True, check=False)
    duration = round(time.perf_counter() - started, 3)
    finished_at = utc_now()
    metadata = {
        "command": command,
        "started_at_utc": started_at,
        "finished_at_utc": finished_at,
        "duration_seconds": duration,
        "returncode": result.returncode,
        "stdout": decode_output(result.stdout),
        "stderr": decode_output(result.stderr),
        "executable": str(test_exe),
        "executable_sha256": sha256_bytes(test_exe.read_bytes()),
        "build": build_metadata,
    }
    if result.returncode != 0:
        raise RuntimeError(
            f"headless test suite failed with exit code {result.returncode}: {metadata['stderr']}"
        )
    return metadata


def parse_genuine_telemetry(csv_path):
    """Parse genuine C-engine telemetry rows without fabricating samples."""
    with csv_path.open("r", encoding="utf-8", newline="") as source:
        reader = csv.DictReader(source)
        if not reader.fieldnames:
            raise ValueError(f"Telemetry CSV has no header: {csv_path}")
        return reader.fieldnames, list(reader)


def float_value(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except (TypeError, ValueError):
        return default


def rounded(value, digits):
    return None if value is None else round(value, digits)


def build_derived_physics(row, prev_row=None):
    """Compute physically named metrics from raw engine telemetry."""
    first_row = prev_row is None
    if prev_row is None:
        prev_row = row

    time_s = float_value(row, "time_s")
    prev_time_s = float_value(prev_row, "time_s")
    dt = time_s - prev_time_s if time_s > prev_time_s else 0.008333333

    speed = float_value(row, "speed_mps")
    prev_speed = float_value(prev_row, "speed_mps")
    v_lat = float_value(row, "velocity_lateral_mps")
    prev_v_lat = float_value(prev_row, "velocity_lateral_mps")
    yaw_rate = float_value(row, "yaw_rate_rad_s")
    prev_yaw_rate = float_value(prev_row, "yaw_rate_rad_s")

    accel_long = float_value(row, "solved_long_accel_mps2")
    accel_lat = float_value(row, "lateral_accel_mps2")
    if "solved_long_accel_mps2" not in row:
        accel_long = (speed - prev_speed) / dt
    if "lateral_accel_mps2" not in row:
        accel_lat = (v_lat - prev_v_lat) / dt
    prev_accel_long = 0.0 if first_row else float_value(prev_row, "solved_long_accel_mps2")
    prev_accel_lat = 0.0 if first_row else float_value(prev_row, "lateral_accel_mps2")
    jerk_long = (accel_long - prev_accel_long) / dt if not first_row else 0.0
    jerk_lat = (accel_lat - prev_accel_lat) / dt if not first_row else 0.0
    accel_yaw = (yaw_rate - prev_yaw_rate) / dt

    f_omega = float_value(row, "front_wheel_omega_rad_s")
    r_omega = float_value(row, "rear_wheel_omega_rad_s")
    f_slip_a = float_value(row, "front_slip_angle_rad")
    r_slip_a = float_value(row, "rear_slip_angle_rad")
    f_slip_r = float_value(row, "front_slip_ratio")
    r_slip_r = float_value(row, "rear_slip_ratio")
    f_load = float_value(row, "front_normal_load_n")
    r_load = float_value(row, "rear_normal_load_n")
    static_f = float_value(row, "static_front_load_n")
    static_r = float_value(row, "static_rear_load_n")
    f_fx_lim = float_value(row, "front_fx_limited_n")
    r_fx_lim = float_value(row, "rear_fx_limited_n")
    f_fy_lim = float_value(row, "front_fy_limited_n")
    r_fy_lim = float_value(row, "rear_fy_limited_n")
    f_brake_tq = float_value(row, "front_brake_torque_nm")
    r_brake_tq = float_value(row, "rear_brake_torque_nm")
    handbrake_tq = float_value(row, "handbrake_torque_nm")
    drive_tq = float_value(row, "drive_torque_nm")
    yaw_tq = float_value(row, "yaw_torque_nm")
    f_usage = float_value(row, "front_friction_usage")
    r_usage = float_value(row, "rear_friction_usage")
    steer = float_value(row, "steering_angle_rad")
    sideslip = float_value(row, "body_sideslip_rad")

    total_load = f_load + r_load or 1.0
    service_brake_tq = f_brake_tq + r_brake_tq
    total_brake_tq = service_brake_tq + handbrake_tq
    total_friction_force_n = math.hypot(f_fx_lim + r_fx_lim, f_fy_lim + r_fy_lim)
    front_tire_force_mag_n = math.hypot(f_fx_lim, f_fy_lim)
    rear_tire_force_mag_n = math.hypot(r_fx_lim, r_fy_lim)
    curvature = yaw_rate / speed if speed > 0.1 else 0.0
    turning_radius = 1.0 / curvature if abs(curvature) > 0.0001 else None
    cornering_stiffness = f_fy_lim / f_slip_a if abs(f_slip_a) > 0.001 else None
    stopping_distance = speed**2 / (2.0 * abs(accel_long)) if accel_long < -0.1 else 0.0
    yaw_inertia = yaw_tq / accel_yaw if abs(accel_yaw) > 0.01 else None
    aero_drag = float_value(row, "aero_drag_n")
    rolling_resistance = float_value(row, "rolling_resistance_n")
    rear_wheel_power_kw = (drive_tq * r_omega) / 1000.0

    return {
        "accel_longitudinal_mps2": rounded(accel_long, 3),
        "accel_lateral_mps2": rounded(accel_lat, 3),
        "accel_yaw_rad_s2": rounded(accel_yaw, 3),
        "wheel_speeds_kmh": {
            "front": rounded(f_omega * WHEEL_RADIUS_M * 3.6, 2),
            "rear": rounded(r_omega * WHEEL_RADIUS_M * 3.6, 2),
        },
        "avg_slip_ratio": rounded((abs(f_slip_r) + abs(r_slip_r)) / 2.0, 5),
        "total_brake_force_n": rounded(abs(total_brake_tq) / WHEEL_RADIUS_M, 2),
        "weight_distribution_pct": {
            "front": rounded(f_load / total_load * 100.0, 2),
            "rear": rounded(r_load / total_load * 100.0, 2),
        },
        "load_transfer_ratios": {
            "front": rounded(f_load / static_f if static_f > 0 else None, 4),
            "rear": rounded(r_load / static_r if static_r > 0 else None, 4),
        },
        "total_friction_force_n": rounded(total_friction_force_n, 2),
        "axle_lateral_forces_n": {
            "front": rounded(f_fy_lim, 2),
            "rear": rounded(r_fy_lim, 2),
        },
        "service_brake_balance_pct": rounded(
            f_brake_tq / service_brake_tq * 100.0 if service_brake_tq else 0.0, 2
        ),
        "rear_wheelspin_excess_rad_s": rounded(r_omega - f_omega, 3),
        "rear_wheel_power_estimate_kw": rounded(rear_wheel_power_kw, 3),
        "friction_utilization_pct": rounded((f_usage + r_usage) / 2.0 * 100.0, 2),
        "front_tire_force_magnitude_n": rounded(front_tire_force_mag_n, 2),
        "rear_tire_force_magnitude_n": rounded(rear_tire_force_mag_n, 2),
        "yaw_torque_per_rate_estimate": rounded(
            yaw_tq / yaw_rate if abs(yaw_rate) > 0.01 else None, 3
        ),
        "jerk_longitudinal_mps3": rounded(jerk_long, 3),
        "jerk_lateral_mps3": rounded(jerk_lat, 3),
        "path_curvature_1m": rounded(curvature, 6),
        "turning_radius_m": rounded(turning_radius, 3),
        "friction_ellipse_utilization": {
            # friction_usage is already the normalized combined-force ellipse utilization;
            # slip ratio is a kinematic slip quantity and must not be combined with it.
            "front": rounded(f_usage, 5),
            "rear": rounded(r_usage, 5),
        },
        "slip_energy_w": {
            "front": rounded(abs(f_fx_lim * f_slip_r * speed), 2),
            "rear": rounded(abs(r_fx_lim * r_slip_r * speed), 2),
        },
        "weight_transfer_rate_n_s": {
            "front": rounded((f_load - float_value(prev_row, "front_normal_load_n")) / dt, 2),
            "rear": rounded((r_load - float_value(prev_row, "rear_normal_load_n")) / dt, 2),
        },
        "front_axle_load_delta_from_static_n": rounded(f_load - static_f, 2),
        "rear_axle_load_delta_from_static_n": rounded(r_load - static_r, 2),
        "instantaneous_yaw_inertia_estimate_kg_m2": rounded(yaw_inertia, 3),
        "assumed_front_brake_pressure_kpa": rounded(f_brake_tq * 0.15, 3),
        "assumed_rear_brake_pressure_kpa": rounded(r_brake_tq * 0.15, 3),
        "assumed_drivetrain_loss_nm": rounded(drive_tq * 0.12, 3),
        "static_load_minus_net_tire_force_n": rounded(
            static_f + static_r - total_friction_force_n, 2
        ),
        "stopping_distance_estimate_m": rounded(stopping_distance, 3),
        "front_cornering_stiffness_n_rad": rounded(cornering_stiffness, 3),
        "estimated_tire_temperature_c": rounded(
            22.0 + (abs(f_fx_lim * f_slip_r * speed) + abs(r_fx_lim * r_slip_r * speed)) * 0.002, 3
        ),
        "accel_magnitude_mps2": rounded(math.hypot(accel_long, accel_lat), 3),
        "g_force_long": rounded(accel_long / 9.80665, 5),
        "g_force_lat": rounded(accel_lat / 9.80665, 5),
        "g_force_total": rounded(math.hypot(accel_long, accel_lat) / 9.80665, 5),
        "wheel_speed_differential_rad_s": rounded(abs(f_omega - r_omega), 3),
        "front_slip_angle_deg": rounded(math.degrees(f_slip_a), 3),
        "rear_slip_angle_deg": rounded(math.degrees(r_slip_a), 3),
        "avg_slip_angle_deg": rounded(
            (abs(math.degrees(f_slip_a)) + abs(math.degrees(r_slip_a))) / 2.0, 3
        ),
        "yaw_rate_deg_s": rounded(math.degrees(yaw_rate), 3),
        "steering_angle_deg": rounded(math.degrees(steer), 3),
        "body_slip_angle_deg": rounded(math.degrees(sideslip), 3),
        "aerodynamic_drag_n": rounded(aero_drag, 3),
        "rolling_resistance_n": rounded(rolling_resistance, 3),
        "tire_scrub_velocity_mps": rounded(abs(r_slip_r * speed), 3),
        "front_axle_force_magnitude_n": rounded(front_tire_force_mag_n, 2),
        "rear_axle_force_magnitude_n": rounded(rear_tire_force_mag_n, 2),
        "front_axle_torque_nm": rounded(f_fx_lim * WHEEL_RADIUS_M, 3),
        "rear_axle_torque_nm": rounded(r_fx_lim * WHEEL_RADIUS_M, 3),
        "total_axle_torque_nm": rounded((f_fx_lim + r_fx_lim) * WHEEL_RADIUS_M, 3),
        "lat_accel_mps2": rounded(accel_lat, 3),
    }


def typed_raw_value(field, value):
    if field in INT_FIELDS:
        try:
            return int(value)
        except (TypeError, ValueError):
            return value
    try:
        return float(value)
    except (TypeError, ValueError):
        return value


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def build_input_event_trace(rows):
    controls = ("steering_input", "throttle_input", "brake_input", "handbrake_input")
    events = []
    if not rows:
        return events
    for control in controls:
        events.append(
            {
                "time_s": float_value(rows[0], "time_s"),
                "tick": int(float_value(rows[0], "tick")),
                "type": "initial_control",
                "control": control,
                "from": None,
                "to": float_value(rows[0], control),
            }
        )
    previous_sides = {"steering_angle_rad": 0}
    previous_flags = {"front_locked": 0, "rear_locked": 0}
    for index in range(1, len(rows)):
        row = rows[index]
        previous = rows[index - 1]
        for control in controls:
            before = float_value(previous, control)
            after = float_value(row, control)
            if abs(after - before) > 1e-6:
                events.append(
                    {
                        "time_s": float_value(row, "time_s"),
                        "tick": int(float_value(row, "tick")),
                        "type": "control_change",
                        "control": control,
                        "from": before,
                        "to": after,
                    }
                )
        steer = float_value(row, "steering_angle_rad")
        side = 1 if steer > 0.01 else -1 if steer < -0.01 else 0
        if side != previous_sides["steering_angle_rad"]:
            events.append(
                {
                    "time_s": float_value(row, "time_s"),
                    "tick": int(float_value(row, "tick")),
                    "type": "steering_direction_change",
                    "control": "steering_angle_rad",
                    "from": previous_sides["steering_angle_rad"],
                    "to": side,
                    "value": steer,
                }
            )
            previous_sides["steering_angle_rad"] = side
        for flag in previous_flags:
            before = int(float_value(previous, flag))
            after = int(float_value(row, flag))
            if after != before:
                events.append(
                    {
                        "time_s": float_value(row, "time_s"),
                        "tick": int(float_value(row, "tick")),
                        "type": "lock_state_change",
                        "control": flag,
                        "from": before,
                        "to": after,
                    }
                )
                previous_flags[flag] = after
    return events


def metric_summary(rows, field, transform=lambda value: value):
    values = [transform(float_value(row, field)) for row in rows]
    if not values:
        return {"min": None, "max": None, "mean": None, "peak_time_s": None}
    max_index = max(range(len(values)), key=lambda index: abs(values[index]))
    return {
        "min": round(min(values), 6),
        "max": round(max(values), 6),
        "mean": round(sum(values) / len(values), 6),
        "peak_absolute_value": round(values[max_index], 6),
        "peak_time_s": round(float_value(rows[max_index], "time_s"), 6),
    }


def build_motion_metrics(rows):
    return {
        "speed_mps": metric_summary(rows, "speed_mps"),
        "solved_long_accel_mps2": metric_summary(rows, "solved_long_accel_mps2"),
        "lateral_accel_mps2": metric_summary(rows, "lateral_accel_mps2"),
        "yaw_rate_rad_s": metric_summary(rows, "yaw_rate_rad_s"),
        "body_sideslip_rad": metric_summary(rows, "body_sideslip_rad"),
        "front_slip_angle_rad": metric_summary(rows, "front_slip_angle_rad"),
        "rear_slip_angle_rad": metric_summary(rows, "rear_slip_angle_rad"),
        "front_friction_usage": metric_summary(rows, "front_friction_usage"),
        "rear_friction_usage": metric_summary(rows, "rear_friction_usage"),
        "front_normal_load_n": metric_summary(rows, "front_normal_load_n"),
        "rear_normal_load_n": metric_summary(rows, "rear_normal_load_n"),
        "yaw_torque_nm": metric_summary(rows, "yaw_torque_nm"),
    }


def svg_polyline(values, x, y, width, height, minimum, maximum):
    if not values:
        return ""
    span = maximum - minimum or 1.0
    points = []
    for index, value in enumerate(values):
        px = x + (width * index / max(1, len(values) - 1))
        py = y + height - ((value - minimum) / span) * height
        points.append(f"{px:.1f},{py:.1f}")
    return " ".join(points)


def write_review_svg(target_dir, scenario, rows):
    width, height = 1200, 900
    panels = [
        ("Speed and acceleration", [("speed km/h", "#4cc9f0"), ("longitudinal m/s²", "#f72585")]),
        ("Yaw and sideslip", [("yaw rad/s", "#4cc9f0"), ("sideslip rad", "#f72585")]),
        ("Tire slip angles", [("front deg", "#4cc9f0"), ("rear deg", "#f72585")]),
        (
            "Driver inputs",
            [("throttle", "#4cc9f0"), ("brake", "#f72585"), ("handbrake", "#ffd166")],
        ),
    ]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#10131a"/>',
        f'<text x="40" y="34" fill="#ffffff" font-family=" sans-serif" font-size="22">{scenario} telemetry review</text>',
    ]
    panel_w, panel_h = 540, 365
    for panel_index, (title, series_defs) in enumerate(panels):
        x = 35 + (panel_index % 2) * 575
        y = 55 + (panel_index // 2) * 410
        # Build each plotted series directly from the raw telemetry rows.
        field_map = {
            "speed km/h": lambda r: float_value(r, "speed_mps") * 3.6,
            "longitudinal m/s²": lambda r: float_value(r, "solved_long_accel_mps2"),
            "yaw rad/s": lambda r: float_value(r, "yaw_rate_rad_s"),
            "sideslip rad": lambda r: float_value(r, "body_sideslip_rad"),
            "front deg": lambda r: math.degrees(float_value(r, "front_slip_angle_rad")),
            "rear deg": lambda r: math.degrees(float_value(r, "rear_slip_angle_rad")),
            "throttle": lambda r: float_value(r, "throttle_input"),
            "brake": lambda r: float_value(r, "brake_input"),
            "handbrake": lambda r: float_value(r, "handbrake_input"),
        }
        plotted = [
            (label, [field_map[label](row) for row in rows], color) for label, color in series_defs
        ]
        minimum = min((value for _, values, _ in plotted for value in values), default=0.0)
        maximum = max((value for _, values, _ in plotted for value in values), default=1.0)
        if minimum == maximum:
            minimum -= 1.0
            maximum += 1.0
        svg.append(
            f'<rect x="{x}" y="{y}" width="{panel_w}" height="{panel_h}" rx="8" fill="#181d27" stroke="#343b4a"/>'
        )
        svg.append(
            f'<text x="{x + 16}" y="{y + 28}" fill="#ffffff" font-family="sans-serif" font-size="16">{title}</text>'
        )
        chart_x, chart_y = x + 18, y + 48
        chart_w, chart_h = panel_w - 36, panel_h - 78
        svg.append(
            f'<line x1="{chart_x}" y1="{chart_y + chart_h / 2:.1f}" x2="{chart_x + chart_w}" y2="{chart_y + chart_h / 2:.1f}" stroke="#303746"/>'
        )
        for _label, values, color in plotted:
            svg.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{svg_polyline(values, chart_x, chart_y, chart_w, chart_h, minimum, maximum)}"/>'
            )
        for legend_index, (label, _, color) in enumerate(plotted):
            lx = x + 18 + legend_index * 150
            svg.append(
                f'<rect x="{lx}" y="{y + panel_h - 23}" width="10" height="10" fill="{color}"/>'
            )
            svg.append(
                f'<text x="{lx + 15}" y="{y + panel_h - 13}" fill="#cbd2df" font-family="sans-serif" font-size="11">{label}</text>'
            )
        svg.append(
            f'<text x="{x + panel_w - 120}" y="{y + 48}" fill="#8993a7" font-family="sans-serif" font-size="10">max {maximum:.3f}</text>'
        )
        svg.append(
            f'<text x="{x + panel_w - 120}" y="{y + panel_h - 34}" fill="#8993a7" font-family="sans-serif" font-size="10">min {minimum:.3f}</text>'
        )
    svg.append("</svg>")
    (target_dir / "review.svg").write_text("\n".join(svg) + "\n", encoding="utf-8")


def generate_machine_readable_logs(target_dir, scenario, execution_metadata):
    source_name = discover_scenario_sources().get(scenario)
    if source_name is None:
        raise ValueError(f"Unknown telemetry scenario: {scenario}")
    csv_path = (TELEMETRY_DIR / source_name).resolve()
    if not csv_path.exists():
        raise FileNotFoundError(f"Expected telemetry for {scenario}: {csv_path}")

    source_bytes = csv_path.read_bytes()
    copied_csv = target_dir / "telemetry.csv"
    copied_csv.write_bytes(source_bytes)
    fieldnames, rows = parse_genuine_telemetry(csv_path)
    if not rows:
        raise ValueError(f"Telemetry CSV is empty: {csv_path}")

    derived = {}
    previous = None
    for row in rows:
        metrics = build_derived_physics(row, previous)
        for key, value in metrics.items():
            derived.setdefault(key, []).append(value)
        previous = row

    files = {"telemetry_csv": copied_csv.name}
    for datapoint_name, data_list in derived.items():
        path = target_dir / f"{datapoint_name}.json"
        write_json(path, data_list)
        files[datapoint_name] = path.name

    raw_fields = {}
    for field in fieldnames:
        path = target_dir / f"raw_{field}.json"
        values = [typed_raw_value(field, row.get(field, "")) for row in rows]
        write_json(path, values)
        raw_fields[field] = path.name

    input_trace_path = target_dir / "input_event_trace.json"
    write_json(input_trace_path, build_input_event_trace(rows))
    files["input_event_trace"] = input_trace_path.name

    motion_path = target_dir / "motion_metrics.json"
    write_json(motion_path, build_motion_metrics(rows))
    files["motion_metrics"] = motion_path.name

    write_review_svg(target_dir, scenario, rows)
    files["review_svg"] = "review.svg"

    times = [float_value(row, "time_s") for row in rows]
    duration = times[-1] if times else 0.0
    sample_rate = (
        (len(rows) - 1) / (times[-1] - times[0]) if len(rows) > 1 and times[-1] > times[0] else None
    )
    nonfinite_raw_values = sum(
        1
        for row in rows
        for field in fieldnames
        if isinstance((value := typed_raw_value(field, row.get(field, ""))), float)
        and not math.isfinite(value)
    )
    quality = {
        "row_count": len(rows),
        "field_count": len(fieldnames),
        "sample_rate_hz": round(sample_rate, 6) if sample_rate is not None else None,
        "time_monotonic": all(b > a for a, b in zip(times, times[1:])),
        "max_backlog_drops": max(int(float_value(row, "backlog_drops")) for row in rows),
        "max_substep_count": max(int(float_value(row, "substep_count")) for row in rows),
        "nonfinite_raw_values": nonfinite_raw_values,
        "friction_usage_over_one": sum(
            1
            for row in rows
            if max(
                float_value(row, "front_friction_usage"), float_value(row, "rear_friction_usage")
            )
            > 1.000001
        ),
    }
    summary = {
        "scenario": scenario,
        "source_telemetry_file": source_name,
        "duration_seconds": round(duration, 6),
        "total_telemetry_rows": len(rows),
        "peak_ground_truth_metrics": {
            "max_speed_kmh": round(max(float_value(row, "speed_mps") for row in rows) * 3.6, 3),
            "max_engine_rpm": round(max(float_value(row, "engine_rpm") for row in rows), 3),
            "max_abs_rear_slip_angle_deg": round(
                max(abs(math.degrees(float_value(row, "rear_slip_angle_rad"))) for row in rows), 3
            ),
            "max_abs_body_sideslip_deg": round(
                max(abs(math.degrees(float_value(row, "body_sideslip_rad"))) for row in rows), 3
            ),
            "max_abs_yaw_rate_deg_s": round(
                max(abs(math.degrees(float_value(row, "yaw_rate_rad_s"))) for row in rows), 3
            ),
        },
        "quality": quality,
    }
    summary_path = target_dir / "telemetry_summary.json"
    write_json(summary_path, summary)
    files["telemetry_summary"] = summary_path.name

    execution_path = target_dir / "execution.json"
    execution = dict(execution_metadata)
    execution["scenario"] = scenario
    execution["source_csv_sha256"] = sha256_bytes(source_bytes)
    execution["copied_csv_sha256"] = sha256_bytes(copied_csv.read_bytes())
    execution["byte_exact_csv_snapshot"] = (
        execution["source_csv_sha256"] == execution["copied_csv_sha256"]
    )
    write_json(execution_path, execution)
    files["execution"] = execution_path.name

    index = {
        "run_info": {
            "log_directory": str(target_dir),
            "scenario": scenario,
            "source_telemetry_file": source_name,
            "execution_mode": "headless C physics engine; no window, audio, display, or raylib call",
            "total_telemetry_rows": len(rows),
            "duration_seconds": round(duration, 6),
            "source_csv_sha256": execution["source_csv_sha256"],
            "copied_csv_sha256": execution["copied_csv_sha256"],
            "byte_exact_csv_snapshot": execution["byte_exact_csv_snapshot"],
        },
        "files": files,
        "raw_fields": raw_fields,
    }
    write_json(target_dir / "index.json", index)
    return index


def main():
    parser = argparse.ArgumentParser(
        description="Headless C telemetry recorder and review artifact generator"
    )
    parser.add_argument("--scenario", default=None, help="exact emitted telemetry scenario name")
    parser.add_argument(
        "--all",
        action="store_true",
        help="record every telemetry-producing scenario after one test-suite run",
    )
    args = parser.parse_args()
    if not args.all and args.scenario is None:
        args.scenario = "scenario_step-steer"

    try:
        execution = run_headless_simulation()
        sources = discover_scenario_sources()
        scenarios = sorted(sources) if args.all else [args.scenario]
        missing = [scenario for scenario in scenarios if scenario not in sources]
        if missing:
            raise ValueError(f"Telemetry scenarios were not emitted: {', '.join(missing)}")
        for scenario in scenarios:
            target_dir = get_next_log_dir()
            index = generate_machine_readable_logs(target_dir, scenario, execution)
            print(
                f"Recorded {scenario}: {index['run_info']['total_telemetry_rows']} rows, "
                f"{index['run_info']['duration_seconds']} s -> {target_dir}"
            )
    except (FileNotFoundError, RuntimeError, ValueError, OSError) as error:
        print(f"record_gameplay.py: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
