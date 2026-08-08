/*
 * run_report.c — the writer declared in run_report.h.
 *
 * One fprintf per JSON line, so the file is readable both as a diff target and by eye. Field
 * order matches docs/design/VALIDATION_SCHEMA.md. Numbers are emitted with enough precision
 * that two runs of the same inputs produce identical bytes (metrics are doubles from the
 * reducer; the deterministic telemetry is what makes them match).
 */
#include "game/run_report.h"

#include <stdio.h>
#include <string.h>

#include "platform/build_info.h"

#define RUN_SCHEMA_VERSION "1.0.0"

const char *run_status_label(RunStatus s)
{
    return (s == RUN_PASS) ? "PASS" : "FAIL";
}

const char *run_failure_reason(RunStatus s)
{
    switch (s) {
        case RUN_PASS: return NULL;
        case RUN_FAIL_CHECKPOINT_MISSED: return "checkpoint_missed";
        case RUN_FAIL_CHECKPOINT_OUT_OF_ORDER: return "checkpoint_out_of_order";
        case RUN_FAIL_STALLED: return "stalled";
        case RUN_FAIL_TICK_BUDGET_EXCEEDED: return "tick_budget_exceeded";
        case RUN_FAIL_INVALID_STATE: return "invalid_state";
        case RUN_FAIL_VIDEO_ENCODE_FAILED: return "video_encode_failed";
        case RUN_FAIL_SPEC_INVALID: return "spec_invalid";
    }
    return "unknown";
}

/* Minimal JSON string escaping, byte-for-byte the same rule as failure_bundle.c's helper so the
 * two writers cannot disagree about what a safe string is. */
static void write_json_string(FILE *out, const char *text)
{
    fputc('"', out);
    for (size_t i = 0; text != NULL && text[i] != '\0'; i++) {
        const unsigned char c = (unsigned char)text[i];
        switch (c) {
            case '"': fputs("\\\"", out); break;
            case '\\': fputs("\\\\", out); break;
            case '\n': fputs("\\n", out); break;
            case '\r': fputs("\\r", out); break;
            case '\t': fputs("\\t", out); break;
            default:
                if (c < 0x20u)
                    fprintf(out, "\\u%04x", c);
                else
                    fputc((int)c, out);
                break;
        }
    }
    fputc('"', out);
}

bool run_report_write(const char *path, const RunReportInput *in)
{
    if (path == NULL || in == NULL) return false;

    FILE *file = fopen(path, "wb");
    if (file == NULL) return false;

    const ValidationMetrics *m = in->metrics;
    const char *reason = run_failure_reason(in->status);

    fprintf(file, "{\n");
    fprintf(file, "  \"schema_version\": \"%s\",\n", RUN_SCHEMA_VERSION);
    fprintf(file, "  \"run_id\": ");
    write_json_string(file, in->runId != NULL ? in->runId : "");
    fprintf(file, ",\n");

    fprintf(file, "  \"result\": { \"status\": \"%s\", \"failure_reason\": ",
            run_status_label(in->status));
    if (reason == NULL)
        fprintf(file, "null");
    else
        write_json_string(file, reason);
    fprintf(file, " },\n");

    fprintf(file, "  \"car\": { \"id\": ");
    write_json_string(file, in->carId != NULL ? in->carId : "");
    fprintf(file, ", \"display_name\": ");
    write_json_string(file, in->carDisplayName != NULL ? in->carDisplayName : "");
    fprintf(file, ", \"drivetrain\": ");
    write_json_string(file, in->carDrivetrain != NULL ? in->carDrivetrain : "");
    fprintf(file, ", \"mass_kg\": %.3f, \"spec_hash\": ", in->carMassKg);
    write_json_string(file, in->carSpecHash != NULL ? in->carSpecHash : "");
    fprintf(file, " },\n");

    fprintf(file, "  \"track\": { \"id\": ");
    write_json_string(file, in->trackId != NULL ? in->trackId : "");
    fprintf(file, ", \"version\": ");
    write_json_string(file, in->trackVersion != NULL ? in->trackVersion : "");
    fprintf(file, ", \"geometry_hash\": ");
    write_json_string(file, in->trackGeometryHash != NULL ? in->trackGeometryHash : "");
    fprintf(file,
            ", \"checkpoint_count\": %d, \"length_m\": %.3f, "
            "\"start_checkpoint_index\": %d },\n",
            in->trackCheckpointCount, in->trackLengthM, in->startCheckpointIndex);

    fprintf(file,
            "  \"sim\": { \"fixed_hz\": %d, \"telemetry_hz\": %d, \"video_fps\": %d, "
            "\"build_commit\": ",
            in->fixedHz, in->telemetryHz, in->videoFps);
    write_json_string(file, in->buildCommit != NULL ? in->buildCommit : DRIFTY_BUILD_COMMIT);
    fprintf(file, ", \"build_dirty\": %s, \"final_state_checksum\": ",
            in->buildDirty ? "true" : "false");
    write_json_string(file, in->finalStateChecksum != NULL ? in->finalStateChecksum : "");
    fprintf(file, ", \"tick_budget\": %d, \"ticks_run\": %d },\n", in->tickBudget,
            in->ticksRun);

    fprintf(file,
            "  \"lap\": { \"out_lap_time_s\": %.6f, \"timed_lap_time_s\": %.6f, "
            "\"timed_lap_times_s\": [",
            m != NULL ? m->outLapTimeS : 0.0, m != NULL ? m->timedLapTimeS : 0.0);
    for (int i = 0; i < VALIDATION_TIMED_LAPS; i++) {
        fprintf(file, "%s%.6f", (i > 0) ? ", " : "", m != NULL ? m->timedLapTimesS[i] : 0.0);
    }
    fprintf(file,
            "], \"timed_laps_completed\": %d, \"best_timed_lap_time_s\": %.6f, "
            "\"mean_timed_lap_time_s\": %.6f, "
            "\"checkpoints_passed\": %d, \"checkpoints_missed\": %d, \"out_of_order_events\": "
            "%d },\n",
            m != NULL ? m->timedLapsCompleted : 0, m != NULL ? m->bestTimedLapTimeS : 0.0,
            m != NULL ? m->meanTimedLapTimeS : 0.0, in->checkpointsPassed,
            in->checkpointsMissed, in->outOfOrderEvents);

    if (m != NULL) {
        fprintf(file, "  \"metrics\": {\n");
        fprintf(file, "    \"max_speed_mps\": %.6f,\n", m->maxSpeedMps);
        fprintf(file, "    \"mean_speed_mps\": %.6f,\n", m->meanSpeedMps);
        fprintf(file, "    \"median_speed_mps\": %.6f,\n", m->medianSpeedMps);
        fprintf(file, "    \"min_moving_speed_mps\": %.6f,\n", m->minMovingSpeedMps);
        fprintf(file, "    \"p05_speed_mps\": %.6f,\n", m->p05SpeedMps);
        fprintf(file, "    \"p95_speed_mps\": %.6f,\n", m->p95SpeedMps);
        fprintf(file, "    \"time_below_5mps_s\": %.6f,\n", m->timeBelow5mpsS);
        fprintf(file, "    \"collisions\": %d,\n", m->collisions);
        fprintf(file, "    \"spin_events\": %d,\n", m->spinEvents);
        fprintf(file, "    \"off_track_events\": %d,\n", m->offTrackEvents);
        fprintf(file, "    \"off_track_time_s\": %.6f,\n", m->offTrackTimeS);
        fprintf(file, "    \"max_abs_sideslip_rad\": %.6f,\n", m->maxAbsSideslipRad);
        fprintf(file, "    \"max_yaw_rate_rad_s\": %.6f,\n", m->maxYawRateRadS);
        fprintf(file, "    \"max_friction_usage\": %.6f,\n", m->maxFrictionUsage);
        fprintf(file, "    \"time_at_friction_limit_s\": %.6f,\n", m->timeAtFrictionLimitS);
        fprintf(file, "    \"max_long_accel_mps2\": %.6f,\n", m->maxLongAccelMps2);
        fprintf(file, "    \"min_long_accel_mps2\": %.6f,\n", m->minLongAccelMps2);
        fprintf(file, "    \"max_abs_lat_accel_mps2\": %.6f,\n", m->maxAbsLatAccelMps2);
        fprintf(file, "    \"row_count\": %d\n", m->rowCount);
        fprintf(file, "  },\n");
    }

    fprintf(file, "  \"artifacts\": { \"telemetry_csv\": \"telemetry.csv\", \"video_mp4\": ");
    write_json_string(file, in->hasVideo ? "run.mp4" : "");
    fprintf(file, ", \"replay\": ");
    write_json_string(file, in->hasReplay ? "replay.txt" : "");
    fprintf(file, " }\n");

    fprintf(file, "}\n");

    fclose(file);
    return true;
}
