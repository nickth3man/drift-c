/*
 * telemetry.h — CSV row writer shared by every headless scenario.
 *
 * Deliberately independent of raylib and of any window: it is plain <stdio.h>, so the
 * headless test executable and (later) the game module can both emit telemetry.
 *
 * TelemetryRow carries the stable physics schema. The header string is generated from the
 * same field list that the row writer formats, so columns and values cannot drift apart.
 *
 * Phase 3 appended the load-transfer, acceleration-filter, and resistance block at the end,
 * leaving every earlier column in place and in order. `front_normal_load_n` and
 * `dynamic_front_load_n` therefore carry the same value: the first is the Phase 2 column
 * name that existing baselines and tools use, the second is the Phase 3 name that reads
 * correctly beside `static_front_load_n`. Both are written rather than one being renamed.
 *
 * Formatting is fixed-precision (%.6f) rather than %g so that byte-for-byte diffs against a
 * committed baseline are meaningful.
 */
#ifndef DRIFTY_TELEMETRY_H
#define DRIFTY_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Stable Phase 2 telemetry schema. */
typedef struct {
    uint64_t tick;
    double timeS;
    float positionXM;
    float positionYM;
    float headingRad;
    float velocityLongitudinalMps;
    float velocityLateralMps;
    float speedMps;
    float yawRateRadS;
    float steeringAngleRad;
    float engineRpm;
    int selectedGear;
    float frontSlipAngleRad;
    float rearSlipAngleRad;
    float frontSlipRatio;
    float rearSlipRatio;
    float frontWheelOmegaRadS;
    float rearWheelOmegaRadS;
    float frontNormalLoadN;
    float rearNormalLoadN;
    float frontFxPureN;
    float rearFxPureN;
    float frontFyPureN;
    float rearFyPureN;
    float frontFxLimitedN;
    float rearFxLimitedN;
    float frontFyLimitedN;
    float rearFyLimitedN;
    float frontFrictionUsage;
    float rearFrictionUsage;
    int frontLocked;
    int rearLocked;
    float driveTorqueNm;
    float frontBrakeTorqueNm;
    float rearBrakeTorqueNm;
    float handbrakeTorqueNm;
    float totalForceXN;
    float totalForceYN;
    float yawTorqueNm;
    float bodySideslipRad;
    float lowSpeedBlend;
    int substepCount;
    int backlogDrops;
    uint32_t stateChecksum;

    /* Phase 3: load transfer, the acceleration filter, and separated resistance. */
    float staticFrontLoadN;
    float staticRearLoadN;
    float dynamicFrontLoadN;
    float dynamicRearLoadN;
    float loadTransferN;
    float previousLongAccelMps2;
    float filteredLongAccelMps2;
    float solvedLongAccelMps2;
    float lateralAccelMps2;
    float aeroDragN;
    float aeroDragXN;
    float aeroDragYN;
    float rollingResistanceN;
    float rollingResistanceXN;
    float rollingResistanceYN;

    /* The driver's held controls, so a report can show what was asked for beside what the
     * car did. */
    float steeringInput;
    float throttleInput;
    float brakeInput;
    float handbrakeInput;

    /* Per-wheel surface identity, appended so existing columns remain stable. */

    int surfaceFrontLeft;
    int surfaceFrontRight;
    int surfaceRearLeft;
    int surfaceRearRight;

    /* Phase 5: lap-validation columns. Appended so every earlier column stays in place and
     * in order; zero for runs that have no checkpointed track, which is why appending them
     * cannot move a committed baseline. The metrics layer (validation_metrics.c) is a pure
     * reducer over these. */
    int checkpointIndex; /* next required gate, or the last one taken */
    int lapIndex;        /* laps completed so far */
    int lapState;        /* 0 out-lap / 1 timed / 2 complete / 3 aborted */
    int checkpointEvent; /* 0 none / 1 in-order / 2 out-of-order / 3 lap-complete */
    float crashLockoutS; /* seconds left in post-impact lockout; rising edge = collision */
    float distanceToCenterlineM; /* nearest-segment distance, signed magnitude */
    int onTrack;                 /* 1 iff all four wheels report the racing surface */
} TelemetryRow;

typedef struct {
    FILE *file;
    long rowCount;
    bool failed; /* set once any write fails; further writes are refused */
} TelemetryWriter;

/* Create dirPath if it does not exist. Returns false if it could not be created and does
 * not already exist. */
bool telemetry_ensure_dir(const char *dirPath);

/* Open path for writing and emit the header row. Returns false on failure, leaving the
 * writer closed and safe to pass to telemetry_close(). */
bool telemetry_open(TelemetryWriter *writer, const char *path);

/* Append one row. Returns false if the writer is not open or the write failed. */
bool telemetry_write_row(TelemetryWriter *writer, const TelemetryRow *row);

/* Flush and close. Returns false if the file was open and closing or any earlier write
 * failed. Safe to call on a writer that was never opened. */
bool telemetry_close(TelemetryWriter *writer);

/* The exact header line (without newline) that telemetry_open() writes. */
const char *telemetry_header(void);

#endif /* DRIFTY_TELEMETRY_H */
