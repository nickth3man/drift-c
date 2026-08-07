/*
 * run_report.h — write the machine-readable run.json for one validation lap.
 *
 * run.json is the contract a handling change is judged against: it carries the car, the track,
 * the build, the lap result, and the full metrics block, so a diff between two runs is a diff
 * between two of these files. The writer is hand-rolled fprintf with JSON escaping, exactly as
 * failure_bundle.c does for summary.json — no JSON library is pulled into the game module.
 *
 * Failure reasons are a closed set so the suite orchestrator can group and count them without
 * parsing free text.
 */
#ifndef DRIFTY_RUN_REPORT_H
#define DRIFTY_RUN_REPORT_H

#include <stdbool.h>

#include "game/validation_metrics.h"

/* The closed set of run outcomes. RUN_PASS is the only one that is not a failure. */
typedef enum {
    RUN_PASS = 0,
    RUN_FAIL_CHECKPOINT_MISSED,
    RUN_FAIL_CHECKPOINT_OUT_OF_ORDER,
    RUN_FAIL_STALLED,
    RUN_FAIL_TICK_BUDGET_EXCEEDED,
    RUN_FAIL_INVALID_STATE,
    RUN_FAIL_VIDEO_ENCODE_FAILED,
    RUN_FAIL_SPEC_INVALID
} RunStatus;

typedef struct {
    /* Identity. runId is caller-built (e.g. "20260807-143201-chicane_v1-rwd_grip"). */
    const char *runId;

    /* Car. specHash/finalStateChecksum are caller-formatted hex strings. */
    const char *carId;
    const char *carDisplayName;
    const char *carDrivetrain; /* "RWD" / "FWD" / "AWD" */
    double carMassKg;
    const char *carSpecHash;

    /* Track. */
    const char *trackId;
    const char *trackVersion;
    const char *trackGeometryHash;
    int trackCheckpointCount;
    double trackLengthM;
    int startCheckpointIndex;

    /* Simulation. */
    int fixedHz;
    int telemetryHz;
    int videoFps;
    const char *buildCommit;
    bool buildDirty;
    const char *finalStateChecksum;
    int tickBudget;
    int ticksRun;

    /* Result + lap accounting. checkpointsMissed is expected (checkpointCount) minus passed. */
    RunStatus status;
    int checkpointsPassed;
    int checkpointsMissed;
    int outOfOrderEvents;

    /* Metrics block. May be NULL only when status is RUN_FAIL_SPEC_INVALID (no run happened). */
    const ValidationMetrics *metrics;

    /* Artifacts present in the same directory as the run.json. */
    bool hasVideo;
    bool hasReplay;
} RunReportInput;

/* "PASS" for the one passing status, "FAIL" otherwise. */
const char *run_status_label(RunStatus s);

/* The closed-set failure token ("checkpoint_missed", ...), or NULL for RUN_PASS. */
const char *run_failure_reason(RunStatus s);

/*
 * Write run.json to `path`. Returns false only when the file could not be opened. The input
 * struct borrows every string; they only need to outlive this call.
 */
bool run_report_write(const char *path, const RunReportInput *in);
#endif /* DRIFTY_RUN_REPORT_H */
