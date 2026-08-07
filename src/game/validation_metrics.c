/*
 * validation_metrics.c — the pure reducer declared in validation_metrics.h.
 *
 * Every threshold here is a literal copy of the Phase 5 definitions, kept as named constants so
 * the run.json and the test can both quote them. The interval detectors walk the row stream once
 * and track the wall-clock start of the current qualifying interval via row.timeS, so the result
 * never depends on the telemetry sampling rate.
 */
#include "game/validation_metrics.h"

#include <math.h>
#include <stdlib.h>

#include <string.h>

#define SPIN_SIDESLIP_RAD 1.48   /* |body sideslip| above this is a spin attitude */
#define SPIN_MIN_SPEED_MPS 2.0   /* ...only while still moving */
#define SPIN_MIN_DURATION_S 0.25 /* ...that must persist this long to count */

#define OFFTRACK_MIN_DURATION_S 0.10 /* all-wheels-off must persist this long to count */

#define FRICTION_LIMIT \
    0.98 /* usage at/above this is "at the limit" (matches physicallySliding) */
#define MIN_MOVING_SPEED_MPS 0.5

static int cmp_double(const void *a, const void *b)
{
    const double da = *(const double *)a;
    const double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

/* Linear-interpolated percentile (0..100) over a sorted array. p50 is the median. */
static double percentile_sorted(const double *sorted, int n, double p)
{
    if (n <= 0) return 0.0;
    if (n == 1) return sorted[0];
    const double rank = (p / 100.0) * (double)(n - 1);
    const int lo = (int)floor(rank);
    const int hi = (int)ceil(rank);
    if (lo == hi) return sorted[lo];
    const double frac = rank - (double)lo;
    return sorted[lo] + frac * (sorted[hi] - sorted[lo]);
}

void validation_metrics_compute(const TelemetryRow *rows, int count, ValidationMetrics *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (rows == NULL || count <= 0) return;

    out->rowCount = count;

    /* Speed statistics. A scratch buffer backs the median and percentiles. */
    double speedSum = 0.0;
    double maxSpeed = -INFINITY;
    double minMoving = INFINITY;
    double *speeds = (double *)malloc(sizeof(double) * (size_t)count);
    /* If malloc fails we still return the non-order-statistic metrics; the order stats stay 0. */
    int speedSamples = 0;

    /* Extremes. */
    double maxAbsSideslip = 0.0;
    double maxYawRate = 0.0;
    double maxFriction = 0.0;
    double timeAtLimit = 0.0;
    double timeBelow5 = 0.0;
    double maxLongAccel = -INFINITY;
    double minLongAccel = INFINITY;
    double maxAbsLatAccel = 0.0;

    /* Checkpoint events. */
    int checkpointsPassed = 0;
    int outOfOrder = 0;

    /* Interval detectors: each holds the timeS at which the qualifying condition began, or -1
     * when the condition is currently false. */
    int spinActive = 0;
    double spinStartS = 0.0;
    int spinEvents = 0;

    int offTrackActive = 0;
    double offTrackStartS = 0.0;
    int offTrackEvents = 0;
    double offTrackTime = 0.0;

    int collisions = 0;
    float prevLockout = 0.0f;

    /* Lap timing comes from lap-completing events recorded in checkpoint_event (== 3). The
     * first is the out-lap finish, the second the timed lap finish. */
    double outLapTime = 0.0;
    double timedLapTime = 0.0;
    int lapCompletesSeen = 0;

    for (int i = 0; i < count; i++) {
        const TelemetryRow *r = &rows[i];
        const double dt = (i > 0) ? (r->timeS - rows[i - 1].timeS) : 0.0;

        const double speed = (double)r->speedMps;
        speedSum += speed;
        if (speed > maxSpeed) maxSpeed = speed;
        if (speeds != NULL) speeds[speedSamples++] = speed;
        if (speed > MIN_MOVING_SPEED_MPS && speed < minMoving) minMoving = speed;
        if (speed < 5.0) timeBelow5 += dt;

        const double absSlip = fabs((double)r->bodySideslipRad);
        if (absSlip > maxAbsSideslip) maxAbsSideslip = absSlip;
        const double absYaw = fabs((double)r->yawRateRadS);
        if (absYaw > maxYawRate) maxYawRate = absYaw;
        const double fu = (double)fmaxf(r->frontFrictionUsage, r->rearFrictionUsage);
        if (fu > maxFriction) maxFriction = fu;
        if (fu >= FRICTION_LIMIT) timeAtLimit += dt;

        const double longA = (double)r->solvedLongAccelMps2;
        if (longA > maxLongAccel) maxLongAccel = longA;
        if (longA < minLongAccel) minLongAccel = longA;
        const double absLat = fabs((double)r->lateralAccelMps2);
        if (absLat > maxAbsLatAccel) maxAbsLatAccel = absLat;

        const int ev = r->checkpointEvent;
        if (ev == 1 || ev == 3) checkpointsPassed++;
        if (ev == 2) outOfOrder++;
        if (ev == 3) {
            lapCompletesSeen++;
            if (lapCompletesSeen == 1)
                outLapTime = r->timeS;
            else if (lapCompletesSeen == 2)
                timedLapTime = r->timeS - outLapTime;
        }

        /* Spin interval: large sideslip while moving. */
        const int spinCond = (absSlip > SPIN_SIDESLIP_RAD && speed > SPIN_MIN_SPEED_MPS);
        if (spinCond && !spinActive) {
            spinActive = 1;
            spinStartS = r->timeS;
        } else if (!spinCond && spinActive) {
            if (r->timeS - spinStartS >= SPIN_MIN_DURATION_S) spinEvents++;
            spinActive = 0;
        }

        /* Off-track interval: onTrack == 0 (all four wheels off the racing surface). */
        const int offCond = (r->onTrack == 0);
        if (offCond && !offTrackActive) {
            offTrackActive = 1;
            offTrackStartS = r->timeS;
        } else if (!offCond && offTrackActive) {
            if (r->timeS - offTrackStartS >= OFFTRACK_MIN_DURATION_S) offTrackEvents++;
            offTrackTime += r->timeS - offTrackStartS;
            offTrackActive = 0;
        } else if (offCond) {
            /* Accumulate ongoing off-track time as we go so a run that ends off-track is counted. */
        }

        /* Collision: rising edge of the crash lockout timer. */
        if (i > 0 && r->crashLockoutS > 0.0f && prevLockout <= 0.0f) collisions++;
        prevLockout = r->crashLockoutS;
    }

    /* Close any interval still open at the end of the run. */
    if (spinActive && count > 0 && (rows[count - 1].timeS - spinStartS) >= SPIN_MIN_DURATION_S)
        spinEvents++;
    if (offTrackActive && count > 0) {
        const double dur = rows[count - 1].timeS - offTrackStartS;
        offTrackTime += dur;
        if (dur >= OFFTRACK_MIN_DURATION_S) offTrackEvents++;
    }

    if (speeds != NULL) {
        qsort(speeds, (size_t)speedSamples, sizeof(double), cmp_double);
    }

    out->outLapTimeS = outLapTime;
    out->timedLapTimeS = timedLapTime;
    out->maxSpeedMps = (maxSpeed == -INFINITY) ? 0.0 : maxSpeed;
    out->meanSpeedMps = speedSum / (double)count;
    out->medianSpeedMps =
        (speeds != NULL) ? percentile_sorted(speeds, speedSamples, 50.0) : 0.0;
    out->minMovingSpeedMps = (minMoving == INFINITY) ? 0.0 : minMoving;
    out->p05SpeedMps = (speeds != NULL) ? percentile_sorted(speeds, speedSamples, 5.0) : 0.0;
    out->p95SpeedMps = (speeds != NULL) ? percentile_sorted(speeds, speedSamples, 95.0) : 0.0;
    out->timeBelow5mpsS = timeBelow5;
    out->checkpointsPassed = checkpointsPassed;
    out->outOfOrderEvents = outOfOrder;
    out->collisions = collisions;
    out->spinEvents = spinEvents;
    out->offTrackEvents = offTrackEvents;
    out->offTrackTimeS = offTrackTime;
    out->maxAbsSideslipRad = maxAbsSideslip;
    out->maxYawRateRadS = maxYawRate;
    out->maxFrictionUsage = maxFriction;
    out->timeAtFrictionLimitS = timeAtLimit;
    out->maxLongAccelMps2 = (maxLongAccel == -INFINITY) ? 0.0 : maxLongAccel;
    out->minLongAccelMps2 = (minLongAccel == INFINITY) ? 0.0 : minLongAccel;
    out->maxAbsLatAccelMps2 = maxAbsLatAccel;

    free(speeds);
}
