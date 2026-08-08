/*
 * validation_metrics.h — pure reduction of a lap run to summary metrics.
 *
 * The run.json this project ships is only as trustworthy as the numbers in it, so the metric
 * computation is a pure function over the accumulated telemetry rows: no Game, no global state,
 * no I/O. Feed it the rows a run collected and it writes one ValidationMetrics. The
 * scenario_run_report test feeds it a hand-built array and checks every field against values it
 * computed by hand, so a regression here is loud.
 *
 * Interval-based event counts (spin, off-track, collision) are the two definitions that used to
 * live in the deleted scoring classifier, rebuilt here as physics + time, never as gameplay
 * rules. They are intervals rather than per-tick booleans so a faster sampling rate cannot
 * inflate them.
 */
#ifndef DRIFTY_VALIDATION_METRICS_H
#define DRIFTY_VALIDATION_METRICS_H

#include "game/telemetry.h"

/* Timed laps a validation run records after its out-lap. Three rather than one because a single
 * lap is a poor instrument: the same driver and car will vary by more than a second between laps
 * as small differences at one corner propagate through the next, so a claim about pace has to
 * rest on a mean, not on whichever lap happened to be measured. */
#define VALIDATION_TIMED_LAPS 3

/* Laps a validation run drives in total: the out-lap plus the timed ones. Derived rather than
 * written down twice, because a run that stopped one lap short of what the metrics expect would
 * report a zero for a lap nobody noticed was missing. */
#define VALIDATION_RUN_LAPS (1 + VALIDATION_TIMED_LAPS)

typedef struct {
    /* Lap timing, in seconds. Zero until the run reaches that phase.
     *
     * timedLapTimeS is the FIRST timed lap, kept under its original name and meaning so an
     * older run.json stays comparable with a newer one. The array holds every timed lap, and
     * best/mean are over the ones actually completed (timedLapsCompleted of them). */
    double outLapTimeS;
    double timedLapTimeS;
    double timedLapTimesS[VALIDATION_TIMED_LAPS];
    double bestTimedLapTimeS;
    double meanTimedLapTimeS;
    int timedLapsCompleted;

    /* Speed, m/s, over the whole run. minMoving ignores ticks below MIN_MOVING_SPEED_MPS so a
     * stationary start does not pin the minimum at zero. */
    double maxSpeedMps;
    double meanSpeedMps;
    double medianSpeedMps;
    double minMovingSpeedMps;
    double p05SpeedMps;
    double p95SpeedMps;
    double timeBelow5mpsS;

    /* Checkpoint accounting. checkpointsPassed counts in-order and lap-completing crossings;
     * outOfOrderEvents counts out-of-order crossings. checkpointsMissed is expected minus
     * passed for the timed lap and is computed by run_report from track metadata, not here. */
    int checkpointsPassed;
    int outOfOrderEvents;

    /* Event counts. collision = rising edges of the crash lockout; spin = sustained large
     * body sideslip while moving; offTrack = sustained all-wheels-off-racing-surface. */
    int collisions;
    int spinEvents;
    int offTrackEvents;
    double offTrackTimeS;

    /* Extremes the diff tool watches after a handling change. */
    double maxAbsSideslipRad;
    double maxYawRateRadS;
    double maxFrictionUsage;
    double timeAtFrictionLimitS;

    double maxLongAccelMps2;
    double minLongAccelMps2;
    double maxAbsLatAccelMps2;

    int rowCount; /* how many rows were reduced, for cross-checks */
} ValidationMetrics;

/*
 * Reduce `rows[0 .. count)` into `out`. Safe on count <= 0 (writes zeros). Pure: allocates a
 * temporary buffer for the median/percentile sort and frees it; touches nothing else.
 */
void validation_metrics_compute(const TelemetryRow *rows, int count, ValidationMetrics *out);

#endif /* DRIFTY_VALIDATION_METRICS_H */
