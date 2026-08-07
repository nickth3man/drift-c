# Drifty Validation Pipeline Schema (`run.json` & `suite.json`)

## Overview

The Milestone 1 validation pipeline evaluates vehicle dynamics and baseline AI driver performance deterministically. Each validation run yields a `run.json` reporting the vehicle spec, track parameters, simulation metadata, lap breakdown, summary metrics, and result status. A validation suite run collects all car runs into a `suite.json`.

---

## 1. `run.json` Schema

### Top-Level Fields

| Field | Type | Description |
|---|---|---|
| `schema_version` | `string` | Version of the run schema (e.g., `"1.0.0"`). |
| `run_id` | `string` | Run identifier containing track, car, and starting checkpoint. |
| `result` | `object` | Status and optional failure token. |
| `car` | `object` | Vehicle identity, spec hash, drivetrain, and mass. |
| `track` | `object` | Track identity, version, geometry hash, gate count, length, and starting checkpoint. |
| `sim` | `object` | Timestep rates, build commit provenance, and final state checksum. |
| `lap` | `object` | Out-lap and timed lap timing and gate accounting. |
| `metrics` | `object` | Detailed summary metrics computed by `validation_metrics.c`. |
| `artifacts` | `object` | Relative paths to telemetry CSV, MP4 video, and input replay binary. |

---

### `result` Object

```json
{
  "status": "PASS",
  "failure_reason": null
}
```

- `status`: `"PASS"` or `"FAIL"`.
- `failure_reason`: `null` when `status` is `"PASS"`. When `status` is `"FAIL"`, it must be one of the closed-set failure tokens:
  - `"checkpoint_missed"`: One or more required gates were skipped.
  - `"checkpoint_out_of_order"`: Gates were crossed in reverse or out-of-order sequence.
  - `"stalled"`: Vehicle speed remained near zero while time elapsed.
  - `"tick_budget_exceeded"`: Failed to complete the required laps within the maximum tick budget.
  - `"invalid_state"`: Non-finite simulation state detected (NaN/Inf in position/velocity).
  - `"video_encode_failed"`: ffmpeg frame pipe failed or executable was missing.
  - `"spec_invalid"`: Vehicle spec failed parameter bounds check.

---

### `car` Object

```json
{
  "id": "rwd_grip",
  "display_name": "RWD Grip",
  "drivetrain": "RWD",
  "mass_kg": 1220.0,
  "spec_hash": "a1b2c3d4"
}
```

- `drivetrain`: `"RWD"`, `"FWD"`, or `"AWD"`.
- `spec_hash`: 8-character hex FNV-1a hash over `VehicleSpec` bytes.

---

### `track` Object

```json
{
  "id": "technical",
  "version": "technical_v1",
  "geometry_hash": "e5f6a7b8",
  "checkpoint_count": 8,
  "length_m": 422.9,
  "start_checkpoint_index": 3
}
```

- `id`: authored validation layout: `chicane`, `sprint`, or the tighter `technical` circuit.
- `version`: changes whenever the authored layout changes.
- `geometry_hash`: 8-character hex FNV-1a hash over node geometry, the learned racing line, and
  checkpoint geometry. It changes when the racing line is regenerated, so a run recorded against
  an older line is identifiable rather than silently comparable.
- `start_checkpoint_index`: ordered gate used for the standing-start pose; the run must still
  complete two laps through the normal checkpoint sequence.


---

### `sim` Object

```json
{
  "fixed_hz": 120,
  "telemetry_hz": 60,
  "video_fps": 60,
  "build_commit": "abc1234",
  "build_dirty": false,
  "final_state_checksum": "11223344",
  "tick_budget": 14400,
  "ticks_run": 8460
}
```

---

### `lap` Object

```json
{
  "out_lap_time_s": 33.100000,
  "timed_lap_time_s": 31.482000,
  "checkpoints_passed": 8,
  "checkpoints_missed": 0,
  "out_of_order_events": 0
}
```

- All lap times are in seconds (`s`). `timed_lap_time_s` is the primary performance metric.

---

### `metrics` Object

```json
{
  "max_speed_mps": 35.210000,
  "mean_speed_mps": 19.840000,
  "median_speed_mps": 18.500000,
  "min_moving_speed_mps": 4.200000,
  "p05_speed_mps": 5.100000,
  "p95_speed_mps": 34.000000,
  "time_below_5mps_s": 1.200000,
  "collisions": 0,
  "spin_events": 0,
  "off_track_events": 0,
  "off_track_time_s": 0.000000,
  "max_abs_sideslip_rad": 0.120000,
  "max_yaw_rate_rad_s": 0.450000,
  "max_friction_usage": 0.992000,
  "time_at_friction_limit_s": 21.400000,
  "max_long_accel_mps2": 4.800000,
  "min_long_accel_mps2": -6.200000,
  "max_abs_lat_accel_mps2": 7.500000,
  "row_count": 4230
}
```

#### Units and Definitions

| Metric | Unit | Definition |
|---|---|---|
| `max_speed_mps` | m/s | Peak vehicle speed. |
| `mean_speed_mps` | m/s | Time-weighted mean speed over entire run. |
| `median_speed_mps` | m/s | Median speed (50th percentile). |
| `min_moving_speed_mps` | m/s | Minimum speed while speed > 0.5 m/s. |
| `p05_speed_mps` | m/s | 5th percentile speed. |
| `p95_speed_mps` | m/s | 95th percentile speed. |
| `time_below_5mps_s` | s | Accumulated time spent below 5.0 m/s. |
| `collisions` | count | Count of rising edges in post-impact lockout timer (`crashLockoutS`). |
| `spin_events` | count | Count of contiguous intervals with `\|bodySideslipRad\| > 1.48 rad` for ≥ 0.25 s while speed > 2 m/s. |
| `off_track_events` | count | Count of contiguous intervals with all 4 wheels off racing surface (`onTrack == 0`) for ≥ 0.1 s. |
| `off_track_time_s` | s | Total accumulated time spent off-track. |
| `max_abs_sideslip_rad` | rad | Peak absolute body sideslip angle. |
| `max_yaw_rate_rad_s` | rad/s | Peak absolute vehicle yaw rate. |
| `max_friction_usage` | ratio | Peak tire friction usage (`max(front, rear)`). |
| `time_at_friction_limit_s` | s | Total time spent with tire friction usage ≥ 0.98. |
| `max_long_accel_mps2` | m/s² | Peak forward longitudinal acceleration. |
| `min_long_accel_mps2` | m/s² | Peak braking/deceleration. |
| `max_abs_lat_accel_mps2` | m/s² | Peak lateral cornering acceleration. |
| `row_count` | count | Total 60 Hz telemetry rows reduced. |

---

## 2. `suite.json` Schema

Aggregates every car/track/start scenario in the suite. The default suite runs all roster cars on
the chicane from checkpoint 0, the sprint layout from checkpoint 3, and the tighter technical
layout from checkpoint 3. Each PASS must report 16 ordered checkpoint crossings: two complete
eight-gate laps relative to its start gate.

```json
{
  "suite_id": "20260807-173442-track-suite_v2-abc1234",
  "scenarios": [
    { "track": "chicane", "start_checkpoint": 0 },
    { "track": "sprint", "start_checkpoint": 3 },
    { "track": "technical", "start_checkpoint": 3 }
  ],
  "commit": "abc1234",
  "total": 18,
  "passed": 18,
  "failed": 0,
  "runs": [
    {
      "case_id": "technical-start3-rwd_grip",
      "car_id": "rwd_grip",
      "track": "technical",
      "start_checkpoint": 3,
      "status": "PASS",
      "failure_reason": null,
      "timed_lap_time_s": 27.158000,
      "run_json": "technical-start3-rwd_grip/rwd_grip/run.json"
    }
  ]
}
```
