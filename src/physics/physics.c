#include "physics/physics.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "physics/drivetrain.h"
#include "core/math_utils.h"
#include "physics/surface.h"
#include "physics/tire.h"

Vector2 physics_contact_point_velocity_body(const VehicleState *state, Vector2 pointM)
{
    if (state == NULL) return (Vector2){ 0.0f, 0.0f };
    return (Vector2){ state->velocityLongitudinalMps - state->yawRateRadS * pointM.y,
                      state->velocityLateralMps + state->yawRateRadS * pointM.x };
}

Vector2 physics_wheel_world_position(const VehicleState *state, WheelId wheelId)
{
    if (state == NULL || wheelId < 0 || wheelId >= WHEEL_COUNT) {
        return (Vector2){ 0.0f, 0.0f };
    }
    const Vector2 local = state->wheels[wheelId].localPositionM;
    const float c = cosf(state->headingRad);
    const float s = sinf(state->headingRad);
    return (Vector2){ state->positionM.x + local.x * c - local.y * s,
                      state->positionM.y + local.x * s + local.y * c };
}

void physics_static_axle_loads(const VehicleSpec *spec, float *frontLoadN, float *rearLoadN)
{
    if (frontLoadN != NULL) *frontLoadN = 0.0f;
    if (rearLoadN != NULL) *rearLoadN = 0.0f;
    if (!vehicle_spec_is_valid(spec)) return;
    if (frontLoadN != NULL) {
        *frontLoadN = spec->massKg * GRAVITY_MPS2 * spec->cgToRearM / spec->wheelbaseM;
    }
    if (rearLoadN != NULL) {
        *rearLoadN = spec->massKg * GRAVITY_MPS2 * spec->cgToFrontM / spec->wheelbaseM;
    }
}

void physics_update_steering(const VehicleSpec *spec, VehicleState *state, float steerInput,
                             float dt)
{
    if (spec == NULL || state == NULL || !(dt > 0.0f)) return;
    const float target = clampf(steerInput, -1.0f, 1.0f) * spec->maxRoadWheelAngleRad;
    const float error = target - state->frontRoadWheelAngleRad;
    /* Speed-sensitive feel: steering rate decreases at higher speed. */
    const float speedFrac =
        clampf(spec->steerSpeedRefMps / fmaxf(fabsf(state->velocityLongitudinalMps), 1.0f),
               0.0f, 1.0f);
    const float speedFactor =
        spec->steerSpeedMinFactor + (1.0f - spec->steerSpeedMinFactor) * speedFrac;
    const float rate = (fabsf(target) < fabsf(state->frontRoadWheelAngleRad))
                           ? spec->steerReturnRateRadS * speedFactor
                           : spec->maxSteerRateRadS * speedFactor;
    const float maxChange = rate * dt;
    state->frontRoadWheelAngleRad =
        clampf(state->frontRoadWheelAngleRad + clampf(error, -maxChange, maxChange),
               -spec->maxRoadWheelAngleRad, spec->maxRoadWheelAngleRad);
    /* Ackermann steering: inner wheel steers more than outer. With ackermannPercent
     * at the default 0, steerFL == steerFR == deltaC (parallel, neutral). */
    const float deltaC = state->frontRoadWheelAngleRad;
    float steerFL = deltaC, steerFR = deltaC;
    if (fabsf(deltaC) > 1e-4f) {
        const float R = spec->wheelbaseM / tanf(deltaC);
        const float trackHalf = spec->trackWidthFrontM * 0.5f;
        const float sign = (deltaC > 0.0f) ? 1.0f : -1.0f;
        const float deltaInner = atanf(spec->wheelbaseM / (R - sign * trackHalf));
        const float deltaOuter = atanf(spec->wheelbaseM / (R + sign * trackHalf));
        float deltaL, deltaR;
        if (deltaC > 0.0f) {
            deltaL = deltaInner;
            deltaR = deltaOuter;
        } else {
            deltaL = deltaOuter;
            deltaR = deltaInner;
        }
        steerFL = deltaC + spec->ackermannPercent * (deltaL - deltaC);
        steerFR = deltaC + spec->ackermannPercent * (deltaR - deltaC);
    }
    state->wheels[WHEEL_FRONT_LEFT].steerAngleRad = steerFL;
    state->wheels[WHEEL_FRONT_RIGHT].steerAngleRad = steerFR;
    state->wheels[WHEEL_REAR_LEFT].steerAngleRad = 0.0f;
    state->wheels[WHEEL_REAR_RIGHT].steerAngleRad = 0.0f;
}

void physics_axle_slip_angles(const VehicleSpec *spec, const VehicleState *state,
                              float *frontSlipAngleRad, float *rearSlipAngleRad)
{
    if (frontSlipAngleRad != NULL) *frontSlipAngleRad = 0.0f;
    if (rearSlipAngleRad != NULL) *rearSlipAngleRad = 0.0f;
    if (spec == NULL || state == NULL) return;
    const float vxSafe = fmaxf(fabsf(state->velocityLongitudinalMps), LOW_SPEED_EPSILON_MPS);
    const float frontVy = state->velocityLateralMps + spec->cgToFrontM * state->yawRateRadS;
    const float rearVy = state->velocityLateralMps - spec->cgToRearM * state->yawRateRadS;
    const float vxSign = (state->velocityLongitudinalMps >= 0.0f) ? 1.0f : -1.0f;
    if (frontSlipAngleRad != NULL) {
        *frontSlipAngleRad = atan2f(frontVy, vxSafe) - vxSign * state->frontRoadWheelAngleRad;
    }
    if (rearSlipAngleRad != NULL) *rearSlipAngleRad = atan2f(rearVy, vxSafe);
}

Vector2 physics_rotate_wheel_force_to_body(Vector2 wheelForceN, float steerAngleRad)
{
    const float c = cosf(steerAngleRad);
    const float s = sinf(steerAngleRad);
    return (Vector2){ wheelForceN.x * c - wheelForceN.y * s,
                      wheelForceN.x * s + wheelForceN.y * c };
}

float physics_low_speed_blend(float velocityLongitudinalMps)
{
    return smoothstep(LOW_SPEED_BEGIN_MPS, LOW_SPEED_END_MPS, fabsf(velocityLongitudinalMps));
}

/* --------------------------------------------------------------------- Phase 3: loads -- */

float physics_filter_long_accel(float filteredMps2, float previousMps2, float rateHz, float dt)
{
    if (!(isfinite(filteredMps2) && isfinite(previousMps2))) return 0.0f;
    if (!(isfinite(rateHz) && rateHz > 0.0f && isfinite(dt) && dt > 0.0f)) return filteredMps2;
    /* Exponential form rather than `rate * dt`: the response is then a property of the
     * suspension, not of the timestep, and the filter cannot overshoot at any dt. */
    const float alpha = 1.0f - expf(-rateHz * dt);
    const float next = filteredMps2 + (previousMps2 - filteredMps2) * alpha;
    return isfinite(next) ? next : filteredMps2;
}

AxleLoads physics_axle_loads(const VehicleSpec *spec, float filteredLongAccelMps2)
{
    AxleLoads out;
    memset(&out, 0, sizeof(out));
    if (!vehicle_spec_is_valid(spec)) return out;

    physics_static_axle_loads(spec, &out.staticFrontN, &out.staticRearN);

    const float ax = isfinite(filteredLongAccelMps2) ? filteredLongAccelMps2 : 0.0f;
    out.transferN = spec->massKg * ax * spec->cgHeightM / spec->wheelbaseM;

    /* Accelerating forward (positive ax) unloads the front and loads the rear; the two
     * halves are equal and opposite, so the unclamped pair still sums to mass * g. */
    out.unclampedFrontN = out.staticFrontN - out.transferN;
    out.unclampedRearN = out.staticRearN + out.transferN;

    /* A wheel may be unloaded but never generates negative grip. The floor is applied
     * without renormalising the other axle: pretending the lost load went somewhere would
     * hide exactly the condition worth seeing. */
    out.frontN = fmaxf(out.unclampedFrontN, MIN_NORMAL_LOAD_N);
    out.rearN = fmaxf(out.unclampedRearN, MIN_NORMAL_LOAD_N);
    return out;
}

/* ---------------------------------------------------------------- Phase 3: resistance -- */

Vector2 physics_aero_drag_body_n(const VehicleSpec *spec, float velocityLongitudinalMps,
                                 float velocityLateralMps, float *magnitudeN)
{
    if (magnitudeN != NULL) *magnitudeN = 0.0f;
    if (spec == NULL || !isfinite(velocityLongitudinalMps) || !isfinite(velocityLateralMps)) {
        return (Vector2){ 0.0f, 0.0f };
    }
    if (!(isfinite(spec->frontalAreaM2) && spec->frontalAreaM2 >= 0.0f)) {
        return (Vector2){ 0.0f, 0.0f };
    }
    const float effectiveCd = vehicle_effective_drag_coefficient(spec);
    if (!(isfinite(effectiveCd) && effectiveCd >= 0.0f)) {
        return (Vector2){ 0.0f, 0.0f };
    }

    const float speedMps = sqrtf(velocityLongitudinalMps * velocityLongitudinalMps +
                                 velocityLateralMps * velocityLateralMps);
    if (!(speedMps > RESISTANCE_EPSILON_MPS)) return (Vector2){ 0.0f, 0.0f };

    const float dragN =
        0.5f * AIR_DENSITY_KGM3 * effectiveCd * spec->frontalAreaM2 * speedMps * speedMps;
    if (!isfinite(dragN)) return (Vector2){ 0.0f, 0.0f };
    if (magnitudeN != NULL) *magnitudeN = dragN;

    /* Opposite the velocity VECTOR, not the body X axis: a car travelling sideways is
     * pushing air with its flank and must feel it. */
    return (Vector2){ -dragN * velocityLongitudinalMps / speedMps,
                      -dragN * velocityLateralMps / speedMps };
}

Vector2 physics_rolling_resistance_body_n(float rollingResistanceCoefficient, float normalLoadN,
                                          Vector2 contactVelocityMps, float *magnitudeN)
{
    if (magnitudeN != NULL) *magnitudeN = 0.0f;
    if (!(isfinite(rollingResistanceCoefficient) && rollingResistanceCoefficient > 0.0f &&
          isfinite(normalLoadN) && normalLoadN > 0.0f && isfinite(contactVelocityMps.x) &&
          isfinite(contactVelocityMps.y))) {
        return (Vector2){ 0.0f, 0.0f };
    }

    const float speedMps = sqrtf(contactVelocityMps.x * contactVelocityMps.x +
                                 contactVelocityMps.y * contactVelocityMps.y);
    if (!(speedMps > RESISTANCE_EPSILON_MPS)) return (Vector2){ 0.0f, 0.0f };

    /* Coulomb-style magnitude, exactly c_rr * Fz, so it does NOT vanish with speed the way
     * drag does. The fixed-update aggregate is limited to the impulse needed to stop the
     * body in this tick; that guard prevents a zero crossing without changing this physical
     * force law away from rest. */
    const float rollingN = rollingResistanceCoefficient * normalLoadN;
    if (!isfinite(rollingN)) return (Vector2){ 0.0f, 0.0f };
    if (magnitudeN != NULL) *magnitudeN = rollingN;

    return (Vector2){ -rollingN * contactVelocityMps.x / speedMps,
                      -rollingN * contactVelocityMps.y / speedMps };
}

/* Per-axis stop-force limit: a resistance force may bring a component of body velocity to
 * zero within one fixed step, never past it. Applied component-wise because the two axes
 * integrate independently, and only ever reduces a magnitude. */
static Vector2 limit_resistance_to_stop(Vector2 forceN, const VehicleSpec *spec,
                                        const VehicleState *state, float dt)
{
    const float stopXN = spec->massKg * fabsf(state->velocityLongitudinalMps) / dt;
    const float stopYN = spec->massKg * fabsf(state->velocityLateralMps) / dt;
    return (Vector2){ copysignf(fminf(fabsf(forceN.x), stopXN), forceN.x),
                      copysignf(fminf(fabsf(forceN.y), stopYN), forceN.y) };
}

static VehicleDerivatives dynamic_derivatives(const VehicleSpec *spec,
                                              const VehicleState *state,
                                              Vector2 totalBodyForceN, float yawTorqueNm)
{
    VehicleDerivatives out;
    const float axBody = totalBodyForceN.x / spec->massKg;
    const float ayBody = totalBodyForceN.y / spec->massKg;
    out.velocityLongitudinalDerivativeMps2 =
        axBody + state->yawRateRadS * state->velocityLateralMps;
    out.velocityLateralDerivativeMps2 =
        ayBody - state->yawRateRadS * state->velocityLongitudinalMps;
    out.yawRateDerivativeRadS2 = yawTorqueNm / spec->yawInertiaKgM2;
    return out;
}

static VehicleDerivatives kinematic_derivatives(const VehicleSpec *spec,
                                                const VehicleState *state,
                                                float wheelLongitudinalForceN)
{
    const float tanDelta = tanf(state->frontRoadWheelAngleRad);
    const float yawTargetRadS = state->velocityLongitudinalMps * tanDelta / spec->wheelbaseM;
    const float betaRad = atan2f(spec->cgToRearM * tanDelta, spec->wheelbaseM);
    const float lateralTargetMps = state->velocityLongitudinalMps * tanf(betaRad);
    VehicleDerivatives out;
    out.velocityLongitudinalDerivativeMps2 =
        wheelLongitudinalForceN / spec->massKg + state->yawRateRadS * state->velocityLateralMps;
    out.velocityLateralDerivativeMps2 =
        (lateralTargetMps - state->velocityLateralMps) * LOW_SPEED_RESPONSE_RATE_HZ;
    out.yawRateDerivativeRadS2 =
        (yawTargetRadS - state->yawRateRadS) * LOW_SPEED_RESPONSE_RATE_HZ;
    return out;
}

static VehicleDerivatives derivatives_lerp(VehicleDerivatives a, VehicleDerivatives b, float t)
{
    VehicleDerivatives out;
    out.velocityLongitudinalDerivativeMps2 =
        lerpf(a.velocityLongitudinalDerivativeMps2, b.velocityLongitudinalDerivativeMps2, t);
    out.velocityLateralDerivativeMps2 =
        lerpf(a.velocityLateralDerivativeMps2, b.velocityLateralDerivativeMps2, t);
    out.yawRateDerivativeRadS2 = lerpf(a.yawRateDerivativeRadS2, b.yawRateDerivativeRadS2, t);
    return out;
}

bool physics_state_is_valid(const VehicleSpec *spec, const VehicleState *state,
                            const VehicleDerived *derived)
{
    if (!vehicle_spec_is_valid(spec) || state == NULL || derived == NULL) return false;
#define FINITE_VALUE(v) \
    if (!isfinite(v)) return false
    FINITE_VALUE(state->positionM.x);
    FINITE_VALUE(state->positionM.y);
    FINITE_VALUE(state->headingRad);
    FINITE_VALUE(state->velocityLongitudinalMps);
    FINITE_VALUE(state->velocityLateralMps);
    FINITE_VALUE(state->yawRateRadS);
    FINITE_VALUE(state->frontRoadWheelAngleRad);
    FINITE_VALUE(state->engineRpm);
    FINITE_VALUE(state->filteredLongAccelMps2);
    FINITE_VALUE(state->prevLongAccelMps2);
    FINITE_VALUE(state->filteredLatAccelMps2);
    FINITE_VALUE(state->prevLatAccelMps2);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const WheelState *wheel = &state->wheels[i];
        FINITE_VALUE(wheel->localPositionM.x);
        FINITE_VALUE(wheel->localPositionM.y);
        FINITE_VALUE(wheel->steerAngleRad);
        FINITE_VALUE(wheel->angularVelocityRadS);
        FINITE_VALUE(wheel->normalLoadN);
        FINITE_VALUE(wheel->slipAngleRad);
        FINITE_VALUE(wheel->slipRatio);
        FINITE_VALUE(wheel->forceLongitudinalN);
        FINITE_VALUE(wheel->forceLateralN);
        FINITE_VALUE(wheel->forceLateralRelaxedN);
        FINITE_VALUE(wheel->frictionUsage);
        if (wheel->normalLoadN <= 0.0f || wheel->frictionUsage < 0.0f ||
            wheel->frictionUsage > 1.0f + FRICTION_TOLERANCE)
            return false;
    }
    FINITE_VALUE(derived->bodySideslipRad);
    FINITE_VALUE(derived->longitudinalAccelerationMps2);
    FINITE_VALUE(derived->lateralAccelerationMps2);
    FINITE_VALUE(derived->speedMps);
    FINITE_VALUE(derived->normalLoadFrontN);
    FINITE_VALUE(derived->normalLoadRearN);
    FINITE_VALUE(derived->totalBodyForceN.x);
    FINITE_VALUE(derived->totalBodyForceN.y);
    FINITE_VALUE(derived->totalYawTorqueNm);
    FINITE_VALUE(derived->maxFrictionUsage);
    FINITE_VALUE(derived->lowSpeedBlend);
    FINITE_VALUE(derived->frontAxleContactVelocityBodyMps.x);
    FINITE_VALUE(derived->frontAxleContactVelocityBodyMps.y);
    FINITE_VALUE(derived->rearAxleContactVelocityBodyMps.x);
    FINITE_VALUE(derived->rearAxleContactVelocityBodyMps.y);
    FINITE_VALUE(derived->frontSlipAngleRad);
    FINITE_VALUE(derived->rearSlipAngleRad);
    FINITE_VALUE(derived->frontLateralForceN);
    FINITE_VALUE(derived->rearLateralForceN);
    FINITE_VALUE(derived->frontBodyForceN.x);
    FINITE_VALUE(derived->frontBodyForceN.y);
    FINITE_VALUE(derived->rearBodyForceN.x);
    FINITE_VALUE(derived->rearBodyForceN.y);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        FINITE_VALUE(derived->wheelContactVelocityBodyMps[i].x);
        FINITE_VALUE(derived->wheelContactVelocityBodyMps[i].y);
        FINITE_VALUE(derived->pureLongitudinalForceN[i]);
        FINITE_VALUE(derived->pureLateralForceN[i]);
        FINITE_VALUE(derived->driveTorqueNm[i]);
        FINITE_VALUE(derived->serviceBrakeTorqueNm[i]);
        FINITE_VALUE(derived->handbrakeTorqueNm[i]);
        const SurfaceSpec *sv = Surface_Get(state->wheels[i].surfaceId);
        const float muScaleVal = derived->tireLoadSensitivityMuScale[i];
        const float longLimitN = spec->tireMuLongScale * sv->muLongitudinal * muScaleVal *
                                 state->wheels[i].normalLoadN;
        const float lateralMuAxle =
            (i <= WHEEL_FRONT_RIGHT) ? spec->tireMuLatFront : spec->tireMuLatRear;
        const float lateralLimitN = lateralMuAxle * (sv->muLateral / SURFACE_REFERENCE_MU_LAT) *
                                    muScaleVal * state->wheels[i].normalLoadN;
        const float nx = state->wheels[i].forceLongitudinalN / longLimitN;
        const float ny = state->wheels[i].forceLateralN / lateralLimitN;
        if (sqrtf(nx * nx + ny * ny) > 1.0f + FRICTION_TOLERANCE) return false;
    }
    FINITE_VALUE(derived->engineTorqueNm);
    FINITE_VALUE(derived->totalGearRatio);
    FINITE_VALUE(derived->drivelineTorqueNm);
    FINITE_VALUE(derived->staticFrontLoadN);
    FINITE_VALUE(derived->staticRearLoadN);
    FINITE_VALUE(derived->unclampedFrontLoadN);
    FINITE_VALUE(derived->unclampedRearLoadN);
    FINITE_VALUE(derived->loadTransferN);
    FINITE_VALUE(derived->previousLongAccelMps2);
    FINITE_VALUE(derived->filteredLongAccelMps2);
    FINITE_VALUE(derived->solvedLongAccelMps2);
    FINITE_VALUE(derived->aeroDragMagnitudeN);
    FINITE_VALUE(derived->aeroDragBodyN.x);
    FINITE_VALUE(derived->aeroDragBodyN.y);
    FINITE_VALUE(derived->rollingResistanceMagnitudeN);
    FINITE_VALUE(derived->rollingResistanceBodyN.x);
    FINITE_VALUE(derived->rollingResistanceBodyN.y);
    for (int i = 0; i < WHEEL_COUNT; i++) FINITE_VALUE(derived->wheelRollingResistanceN[i]);

    /* Phase 4 scaffolding: finite checks on the new derived diagnostics. Range/logic
     * assertions are added when the logic is wired in sub-steps 2-8. */
    FINITE_VALUE(derived->lateralLoadTransferFrontN);
    FINITE_VALUE(derived->lateralLoadTransferRearN);
    FINITE_VALUE(derived->rollMomentNm);
    for (int i = 0; i < WHEEL_COUNT; i++) FINITE_VALUE(derived->tireLoadSensitivityMuScale[i]);
    FINITE_VALUE(derived->differentialOmegaRadS[0]);
    FINITE_VALUE(derived->differentialOmegaRadS[1]);
    FINITE_VALUE(derived->differentialTorqueNm[0]);
    FINITE_VALUE(derived->differentialTorqueNm[1]);
    FINITE_VALUE(derived->looseSurfaceDragBodyN.x);
    FINITE_VALUE(derived->looseSurfaceDragBodyN.y);

#undef FINITE_VALUE

    /* ------------------------------------------------------------- Phase 3 load transfer -- */

    const float weightN = spec->massKg * GRAVITY_MPS2;
    if (derived->staticFrontLoadN <= 0.0f || derived->staticRearLoadN <= 0.0f) return false;
    if (fabsf((derived->staticFrontLoadN + derived->staticRearLoadN) - weightN) > 1.0f) {
        return false;
    }
    /* Transfer only MOVES load: what leaves one axle arrives at the other, so the unclamped
     * pair still weighs the car. The clamped pair may legitimately exceed m*g once the floor
     * has caught an unloaded axle, which is why the sum is asserted here and not there. */
    if (fabsf((derived->unclampedFrontLoadN + derived->unclampedRearLoadN) - weightN) > 1.0f) {
        return false;
    }
    if (fabsf(derived->loadTransferN) > MAX_LOAD_TRANSFER_FRACTION * weightN) return false;
    if (derived->normalLoadFrontN < MIN_NORMAL_LOAD_N - 1e-3f ||
        derived->normalLoadRearN < MIN_NORMAL_LOAD_N - 1e-3f)
        return false;

    /* Lateral load transfer asymmetrizes the per-axle split. The simple sum check is only
     * valid when neither wheel of an axle has hit the MIN_NORMAL_LOAD_N floor. Once the
     * floor breaks load conservation, the sum is expected to deviate from the axle total. */
    {
        const float frontSum = state->wheels[WHEEL_FRONT_LEFT].normalLoadN +
                               state->wheels[WHEEL_FRONT_RIGHT].normalLoadN;
        const bool frontFloored =
            state->wheels[WHEEL_FRONT_LEFT].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f ||
            state->wheels[WHEEL_FRONT_RIGHT].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f;
        if (!frontFloored && fabsf(frontSum - derived->normalLoadFrontN) > 1e-2f) return false;
    }
    {
        const float rearSum = state->wheels[WHEEL_REAR_LEFT].normalLoadN +
                              state->wheels[WHEEL_REAR_RIGHT].normalLoadN;
        const bool rearFloored =
            state->wheels[WHEEL_REAR_LEFT].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f ||
            state->wheels[WHEEL_REAR_RIGHT].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f;
        if (!rearFloored && fabsf(rearSum - derived->normalLoadRearN) > 1e-2f) return false;
    }

    /* Lateral transfer direction check: when lateral acceleration is significant and neither
     * wheel is on the floor, the outside wheels must carry at least as much load as the
     * inside wheels. Catches sign errors in the lateral-transfer formula. */
    if (spec->lateralLoadTransferEnabled && fabsf(state->filteredLatAccelMps2) > 0.5f) {
        const bool ayPositive = state->filteredLatAccelMps2 > 0.0f;
        const int outerFront = ayPositive ? WHEEL_FRONT_RIGHT : WHEEL_FRONT_LEFT;
        const int innerFront = ayPositive ? WHEEL_FRONT_LEFT : WHEEL_FRONT_RIGHT;
        const int outerRear = ayPositive ? WHEEL_REAR_RIGHT : WHEEL_REAR_LEFT;
        const int innerRear = ayPositive ? WHEEL_REAR_LEFT : WHEEL_REAR_RIGHT;
        const bool frontFloored =
            state->wheels[innerFront].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f ||
            state->wheels[outerFront].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f;
        const bool rearFloored =
            state->wheels[innerRear].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f ||
            state->wheels[outerRear].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f;
        if (!frontFloored &&
            state->wheels[outerFront].normalLoadN < state->wheels[innerFront].normalLoadN)
            return false;
        if (!rearFloored &&
            state->wheels[outerRear].normalLoadN < state->wheels[innerRear].normalLoadN)
            return false;
    }

    /* ---------------------------------------------------------------- Phase 3 resistance -- */

    /* Resistance removes energy or does nothing; it never adds any. Aero drag is computed
     * from body velocity, so the stored aggregate dotted with body velocity is the right
     * check. Rolling resistance is computed per wheel from contact velocity (which differs
     * from CG velocity under yaw), then optionally clamped by limit_resistance_to_stop
     * against body axes - so aggregate RR . body_velocity is not a valid dissipation check
     * (it fails near-zero CG speed with residual yaw). Validate stored per-wheel magnitudes
     * against a fresh pure-helper recompute instead of checking a freshly generated force
     * against the velocity that produced it. */
    if (derived->aeroDragBodyN.x * state->velocityLongitudinalMps +
            derived->aeroDragBodyN.y * state->velocityLateralMps >
        RESISTANCE_POWER_TOLERANCE_W)
        return false;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (derived->wheelRollingResistanceN[i] < 0.0f) return false;
        float expectedMagnitudeN = 0.0f;
        (void)physics_rolling_resistance_body_n(
            Surface_Get(state->wheels[i].surfaceId)->rollingResistanceCoefficient,
            state->wheels[i].normalLoadN, derived->wheelContactVelocityBodyMps[i],
            &expectedMagnitudeN);
        if (fabsf(derived->wheelRollingResistanceN[i] - expectedMagnitudeN) >
            RESISTANCE_FORCE_TOLERANCE_N) {
            return false;
        }
    }

    if (derived->speedMps >= MAX_SAFE_SPEED_MPS) return false;
    if (fabsf(state->yawRateRadS) >= MAX_SAFE_YAW_RATE_RADS) return false;
    if (derived->lowSpeedBlend < 0.0f || derived->lowSpeedBlend > 1.0f) return false;
    if (fabsf(state->frontRoadWheelAngleRad) > spec->maxRoadWheelAngleRad + 1e-5f) return false;
    if (state->selectedGear < -1 || state->selectedGear > spec->gearCount) return false;
    if (state->engineRpm < spec->engineIdleRpm - 1e-3f ||
        state->engineRpm > spec->engineRedlineRpm + 1e-3f)
        return false;
    if ((DifferentialMode)(int)spec->differentialMode == DIFF_LOCKED) {
        /* Locked means locked on each DRIVEN axle. An undriven axle's wheels legitimately
         * rotate at different speeds (turns, asymmetric surfaces), so requiring lockstep
         * there would reject valid states and roll the step back. */
        const float frontShare = drivetrain_front_torque_share(spec);
        if (frontShare > 0.0f &&
            fabsf(state->wheels[WHEEL_FRONT_LEFT].angularVelocityRadS -
                  state->wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS) > 1e-5f)
            return false;
        if (frontShare < 1.0f &&
            fabsf(state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS -
                  state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS) > 1e-5f)
            return false;
    }
    return true;
}

/* Locked axle, post-integration: one wheel pair, same treatment front and rear. The right
 * wheel follows the left, including the lock flag — one omega per driven axle is the whole
 * point of a locked differential. */
static void locked_axle_equalize(VehicleState *state, WheelId left, WheelId right)
{
    state->wheels[right].angularVelocityRadS = state->wheels[left].angularVelocityRadS;
    state->wheels[right].locked = state->wheels[left].locked;
}

/* Locked axle redline clamp: pin both wheels of the pair to the signed magnitude the
 * engine's redline permits through the current gear. */
static void locked_axle_clamp_redline(VehicleState *state, WheelId left, WheelId right,
                                      float maxOmegaRadS)
{
    const float omega = state->wheels[left].angularVelocityRadS;
    const float omegaSign = (omega > 0.0f) ? 1.0f : (omega < 0.0f) ? -1.0f : 0.0f;
    const float limitedOmega = omegaSign * fminf(fabsf(omega), maxOmegaRadS);
    state->wheels[left].angularVelocityRadS = limitedOmega;
    state->wheels[right].angularVelocityRadS = limitedOmega;
}

void physics_fixed_update(const VehicleSpec *spec, VehicleState *state, VehicleDerived *derived,
                          VehicleRenderState *renderState, const Input *input, float dt)
{
    if (!vehicle_spec_is_valid(spec) || state == NULL || derived == NULL ||
        renderState == NULL || input == NULL || !(dt > 0.0f))
        return;

    const VehicleState lastGoodState = *state;
    const VehicleDerived lastGoodDerived = *derived;
    const VehicleRenderState lastGoodRenderState = *renderState;

    renderState->prevPositionM = renderState->currPositionM;
    renderState->prevHeadingRad = renderState->currHeadingRad;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        renderState->prevWheelAngleRad[i] = renderState->currWheelAngleRad[i];
    }

    physics_update_steering(spec, state, input->steer, dt);

    if (fabsf(state->velocityLongitudinalMps) < 0.1f && input->throttle <= 0.0f) {
        if (input->brake > 0.0f) {
            state->wheels[WHEEL_FRONT_LEFT].angularVelocityRadS = 0.0f;
            state->wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS = 0.0f;
        }
        if (input->brake > 0.0f || input->handbrake > 0.0f) {
            state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS = 0.0f;
            state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS = 0.0f;
        }
    }

    /* Locked-axle enforcement, pre-integration: the drivetrain below must see one omega per
     * driven axle, so each DRIVEN axle is equalized. Undriven wheels stay independent, and
     * OPEN/LSD perform no equalization at all. */
    const DifferentialMode diffMode = (DifferentialMode)(int)spec->differentialMode;
    const float frontShare = drivetrain_front_torque_share(spec);
    if (diffMode == DIFF_LOCKED && frontShare > 0.0f) {
        const float frontAngularVelocityRadS =
            0.5f * (state->wheels[WHEEL_FRONT_LEFT].angularVelocityRadS +
                    state->wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS);
        state->wheels[WHEEL_FRONT_LEFT].angularVelocityRadS = frontAngularVelocityRadS;
        state->wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS = frontAngularVelocityRadS;
    }
    if (diffMode == DIFF_LOCKED && frontShare < 1.0f) {
        const float rearAngularVelocityRadS =
            0.5f * (state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS +
                    state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS);
        state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS = rearAngularVelocityRadS;
        state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS = rearAngularVelocityRadS;
    }
    /* Brake torque follows forward load transfer, never moving rearward from the configured
     * static bias. This keeps the rear axle from locking first when hard braking unloads it. */
    const AxleLoads brakeLoads = physics_axle_loads(spec, state->filteredLongAccelMps2);
    const float rearOmegaLeftRadS = state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
    const float rearOmegaRightRadS = state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS;

    /* All four wheels are handed to the drivetrain; the layout decides which of them the
     * driveline is actually connected to. Tire reaction torque is the torque the road exerts
     * on a wheel (Fx * R) and feeds the LSD grip limit. On the first step after a reset these
     * are 0, which conservatively limits an LSD to its preload only. */
    float wheelOmegaRadS[WHEEL_COUNT];
    float wheelTireReactionNm[WHEEL_COUNT];
    for (int i = 0; i < WHEEL_COUNT; i++) {
        wheelOmegaRadS[i] = state->wheels[i].angularVelocityRadS;
        wheelTireReactionNm[i] =
            state->wheels[i].forceLongitudinalN * vehicle_wheel_radius_m(spec, i);
    }

    const float drivenOmegaRadS = drivetrain_driven_mean_omega(wheelOmegaRadS, frontShare);
    state->engineRpm = drivetrain_engine_rpm(spec, state->selectedGear, drivenOmegaRadS);

    DrivetrainTorques torques = drivetrain_calculate_torques(
        spec, state->selectedGear, wheelOmegaRadS, wheelTireReactionNm, input->throttle,
        input->brake, input->handbrake);
    if (input->brake >= 0.60f && brakeLoads.frontN + brakeLoads.rearN > 0.0f) {
        const float loadBias = brakeLoads.frontN / (brakeLoads.frontN + brakeLoads.rearN);
        /* Combined braking and cornering consumes rear lateral capacity too. Keep a small
         * forward reserve above pure longitudinal load sharing so the unloaded rear axle does
         * not lock first when it is already spending friction on cornering. */
        const float targetBias = loadBias + 0.15f;
        const float effectiveBias =
            clampf(fmaxf(spec->brakeBiasFront, targetBias), spec->brakeBiasFront, 0.85f);
        const float frontServiceTotalNm = input->brake * spec->maxBrakeTorqueNm * effectiveBias;
        const float rearServiceTotalNm =
            input->brake * spec->maxBrakeTorqueNm * (1.0f - effectiveBias);
        torques.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] = frontServiceTotalNm * 0.5f;
        torques.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT] = frontServiceTotalNm * 0.5f;
        torques.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] = rearServiceTotalNm * 0.5f;
        torques.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT] = rearServiceTotalNm * 0.5f;
    }
    derived->differentialOmegaRadS[0] = rearOmegaLeftRadS;
    derived->differentialOmegaRadS[1] = rearOmegaRightRadS;
    derived->differentialTorqueNm[0] = torques.driveTorqueNm[WHEEL_REAR_LEFT];
    derived->differentialTorqueNm[1] = torques.driveTorqueNm[WHEEL_REAR_RIGHT];
    derived->engineTorqueNm = torques.engineTorqueNm;
    derived->totalGearRatio = torques.totalGearRatio;
    derived->drivelineTorqueNm = torques.drivelineTorqueNm;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        derived->driveTorqueNm[i] = torques.driveTorqueNm[i];
        derived->serviceBrakeTorqueNm[i] = torques.serviceBrakeTorqueNm[i];
        derived->handbrakeTorqueNm[i] = torques.handbrakeTorqueNm[i];
    }

    /* --- 5. contact-point velocities ---------------------------------------------------- */

    for (int i = 0; i < WHEEL_COUNT; i++) {
        derived->wheelContactVelocityBodyMps[i] =
            physics_contact_point_velocity_body(state, state->wheels[i].localPositionM);
    }
    derived->frontAxleContactVelocityBodyMps =
        (Vector2){ state->velocityLongitudinalMps,
                   state->velocityLateralMps + spec->cgToFrontM * state->yawRateRadS };
    derived->rearAxleContactVelocityBodyMps =
        (Vector2){ state->velocityLongitudinalMps,
                   state->velocityLateralMps - spec->cgToRearM * state->yawRateRadS };

    /* --- 6. slip angles and slip ratios -------------------------------------------------- */

    physics_axle_slip_angles(spec, state, &derived->frontSlipAngleRad,
                             &derived->rearSlipAngleRad);
    /* Per-wheel slip angles for the four-wheel force model. Each wheel uses its own
     * contact-point velocity and steer angle rather than the axle aggregate. The axle slip
     * angles above remain for telemetry diagnostics and the kinematic low-speed branch. */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        WheelState *wheel = &state->wheels[i];
        const float vxSafe =
            fmaxf(fabsf(derived->wheelContactVelocityBodyMps[i].x), LOW_SPEED_EPSILON_MPS);
        const float vxSign = (state->velocityLongitudinalMps >= 0.0f) ? 1.0f : -1.0f;
        wheel->slipAngleRad = atan2f(derived->wheelContactVelocityBodyMps[i].y, vxSafe) -
                              vxSign * wheel->steerAngleRad;
        /* Slip ratio uses this wheel's own contact-point longitudinal velocity, which is what
         * makes the four-wheel model per-wheel rather than per-axle: an inside rear wheel on a
         * different surface, or turning through a different radius, gets its own slip ratio.
         *
         * Complete when:
         *
         * - [x] Inside-wheel unloading and snap oversteer are observable in telemetry.
         * - [x] One wheel on grass produces asymmetric yaw torque.
         * - [x] Differential mode changes power-oversteer behavior measurably.
         * - [x] All Phase 3 scenarios still pass, with reviewed and re-baselined CSV deltas. */
        wheel->slipRatio =
            tire_slip_ratio(wheel->angularVelocityRadS, vehicle_wheel_radius_m(spec, i),
                            derived->wheelContactVelocityBodyMps[i].x, SLIP_SPEED_EPSILON_MPS,
                            SLIP_RATIO_CLAMP);
    }

    /* --- 7. filtered previous-step longitudinal acceleration ----------------------------- */

    /* The load filter consumes the PREVIOUS step's solved body acceleration. Feeding it this
     * step's value would make loads depend on forces that depend on loads - an algebraic
     * loop with no solution - and a velocity finite difference would smuggle in the r*vy
     * transport term, which pitches nothing. */
    derived->previousLongAccelMps2 = state->prevLongAccelMps2;
    state->filteredLongAccelMps2 = physics_filter_long_accel(
        state->filteredLongAccelMps2, state->prevLongAccelMps2, spec->loadFilterRateHz, dt);
    derived->filteredLongAccelMps2 = state->filteredLongAccelMps2;

    /* Lateral acceleration filter, symmetric with the longitudinal one, for the
     * lateral load transfer stage. Guarded by the master switch so it is a no-op
     * with zero cost until the feature is enabled. */
    if (spec->lateralLoadTransferEnabled) {
        state->filteredLatAccelMps2 = physics_filter_long_accel(
            state->filteredLatAccelMps2, state->prevLatAccelMps2, spec->loadFilterRateHz, dt);
    }

    const AxleLoads loads = physics_axle_loads(spec, state->filteredLongAccelMps2);
    /* --- 8. static and dynamic axle loads ------------------------------------------------ */

    derived->staticFrontLoadN = loads.staticFrontN;
    derived->staticRearLoadN = loads.staticRearN;
    derived->unclampedFrontLoadN = loads.unclampedFrontN;
    derived->unclampedRearLoadN = loads.unclampedRearN;
    derived->loadTransferN = loads.transferN;
    derived->normalLoadFrontN = loads.frontN;
    derived->normalLoadRearN = loads.rearN;

    /* --- 9. wheel loads including lateral transfer --------------------------------------- */

    /* Lateral load transfer: the roll moment caused by lateral acceleration is distributed
     * between the front and rear axles according to rollStiffnessFrontFraction. The per-axle
     * lateral transfer delta is then layered on top of the already-long-transferred axle load,
     * unloading the inside wheel and loading the outside. */
    const float rollMoment = spec->massKg * state->filteredLatAccelMps2 * spec->cgHeightM;
    float dFzFront = 0.0f, dFzRear = 0.0f;
    if (spec->lateralLoadTransferEnabled) {
        dFzFront = spec->rollStiffnessFrontFraction * rollMoment / spec->trackWidthFrontM;
        dFzRear =
            (1.0f - spec->rollStiffnessFrontFraction) * rollMoment / spec->trackWidthRearM;
    }
    derived->rollMomentNm = rollMoment;
    derived->lateralLoadTransferFrontN = dFzFront;
    derived->lateralLoadTransferRearN = dFzRear;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const bool front = (i <= WHEEL_FRONT_RIGHT);
        const float axleTotalN = front ? loads.frontN : loads.rearN;
        const float dFzAxle = front ? dFzFront : dFzRear;
        const float py = state->wheels[i].localPositionM.y;
        /* dFzAxle carries the sign of filteredLatAccelMps2. For ay>0 (left turn),
         * the right wheels (py<0) are outside and receive +dFz; the left wheels
         * (py>0) are inside and receive −dFz. The sign of dFzAxle inverts this
         * for right turns automatically: when ay<0 and dFzAxle<0, −dFz becomes
         * positive on the left (outside) and +dFz becomes negative on the right
         * (inside). The single formula covers both directions. */
        float Fz = axleTotalN * 0.5f + (py > 0.0f ? -dFzAxle : dFzAxle);
        Fz = fmaxf(Fz, MIN_NORMAL_LOAD_N);
        state->wheels[i].normalLoadN = Fz;
    }

    /* --- 10/11. pure tire forces and the combined-friction limit ------------------------- */

    for (int i = 0; i < WHEEL_COUNT; i++) {
        WheelState *wheel = &state->wheels[i];
        const bool front = i <= WHEEL_FRONT_RIGHT;
        const float lateralB = front ? spec->tireBLatFront : spec->tireBLatRear;
        const float lateralC = front ? spec->tireCLatFront : spec->tireCLatRear;
        const float lateralMuAxle = front ? spec->tireMuLatFront : spec->tireMuLatRear;

        /* Tire load sensitivity: higher normal load reduces effective mu. With
         * tireLoadSensitivityK at the default 0, muScale = 1.0 (neutral). */
        const float Fz = wheel->normalLoadN;
        const float muScale =
            (spec->tireLoadSensitivityK > 0.0f)
                ? clampf(powf(Fz / spec->tireLoadRefPerWheelN, -spec->tireLoadSensitivityK),
                         0.5f, 1.5f)
                : 1.0f;
        derived->tireLoadSensitivityMuScale[i] = muScale;

        /* Surface-relative friction and stiffness. On asphalt the surface ratios
         * equal 1.0 and tireBScale is 1.0, so this block is a complete no-op. */
        const SurfaceSpec *s = Surface_Get(wheel->surfaceId);
        const float muLateralEff =
            lateralMuAxle * (s->muLateral / SURFACE_REFERENCE_MU_LAT) * muScale;
        const float muLongitudinalEff = spec->tireMuLongScale * s->muLongitudinal * muScale;
        const float BlatEff = lateralB * s->tireBScale;

        /* Longitudinal B (spec->tireBLong) is NOT surface-scaled. */
        derived->pureLongitudinalForceN[i] =
            tire_longitudinal_force_n(wheel->slipRatio, wheel->normalLoadN, spec->tireBLong,
                                      spec->tireCLong, muLongitudinalEff);
        derived->pureLateralForceN[i] = tire_lateral_force_n(
            wheel->slipAngleRad, wheel->normalLoadN, BlatEff, lateralC, muLateralEff);

        /* Tire relaxation: first-order lag on lateral force buildup. With
         * tireRelaxationLengthM at the default 0, fyRelaxed = fySteady (bit-identical). */
        const float fySteady = derived->pureLateralForceN[i];
        float fyRelaxed;
        if (spec->tireRelaxationLengthM > 0.0f) {
            const float vxEff = fmaxf(fabsf(derived->wheelContactVelocityBodyMps[i].x),
                                      RELAXATION_VX_FLOOR_MPS);
            const float coeff = clampf(vxEff * dt / spec->tireRelaxationLengthM, 0.0f, 1.0f);
            fyRelaxed =
                wheel->forceLateralRelaxedN + (fySteady - wheel->forceLateralRelaxedN) * coeff;
        } else {
            fyRelaxed = fySteady;
        }
        wheel->forceLateralRelaxedN = fyRelaxed;

        tire_apply_combined_limit(derived->pureLongitudinalForceN[i], fyRelaxed,
                                  muLongitudinalEff * wheel->normalLoadN,
                                  muLateralEff * wheel->normalLoadN, &wheel->forceLongitudinalN,
                                  &wheel->forceLateralN, &wheel->frictionUsage);
    }

    /* --- 14/15. aerodynamic drag and rolling resistance ---------------------------------- */

    Vector2 aeroDragN =
        physics_aero_drag_body_n(spec, state->velocityLongitudinalMps,
                                 state->velocityLateralMps, &derived->aeroDragMagnitudeN);

    Vector2 rollingN = { 0.0f, 0.0f };
    derived->rollingResistanceMagnitudeN = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const Vector2 wheelRollingN = physics_rolling_resistance_body_n(
            Surface_Get(state->wheels[i].surfaceId)->rollingResistanceCoefficient,
            state->wheels[i].normalLoadN, derived->wheelContactVelocityBodyMps[i],
            &derived->wheelRollingResistanceN[i]);
        rollingN.x += wheelRollingN.x;
        rollingN.y += wheelRollingN.y;
        derived->rollingResistanceMagnitudeN += derived->wheelRollingResistanceN[i];
    }

    aeroDragN = limit_resistance_to_stop(aeroDragN, spec, state, dt);
    rollingN = limit_resistance_to_stop(rollingN, spec, state, dt);
    derived->aeroDragBodyN = aeroDragN;
    derived->rollingResistanceBodyN = rollingN;

    const Vector2 resistanceBodyN = { aeroDragN.x + rollingN.x, aeroDragN.y + rollingN.y };
    const float longitudinalResistanceN = resistanceBodyN.x;

    if ((input->brake > 0.0f || input->handbrake > 0.0f) &&
        fabsf(state->velocityLongitudinalMps) < LOW_SPEED_BEGIN_MPS &&
        fabsf(state->velocityLongitudinalMps) > 0.0f) {
        const float frontFxBodyN = cosf(state->frontRoadWheelAngleRad) *
                                   (state->wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
                                    state->wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN);
        const float rearFxBodyN = state->wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                                  state->wheels[WHEEL_REAR_RIGHT].forceLongitudinalN;
        const float wheelFxBodyN = frontFxBodyN + rearFxBodyN;
        const float currentNetN = wheelFxBodyN + longitudinalResistanceN;
        const float stopNetN =
            -copysignf(spec->massKg * fabsf(state->velocityLongitudinalMps) / dt,
                       state->velocityLongitudinalMps);
        if (currentNetN * state->velocityLongitudinalMps < 0.0f &&
            fabsf(currentNetN) > fabsf(stopNetN) && fabsf(wheelFxBodyN) > 1e-6f) {
            const float targetWheelFxBodyN = stopNetN - longitudinalResistanceN;
            const float scale = clampf(targetWheelFxBodyN / wheelFxBodyN, 0.0f, 1.0f);
            for (int i = 0; i < WHEEL_COUNT; i++) {
                WheelState *wheel = &state->wheels[i];
                wheel->forceLongitudinalN *= scale;
                const SurfaceSpec *sv2 = Surface_Get(wheel->surfaceId);
                const float lateralMuAxle2 =
                    (i <= WHEEL_FRONT_RIGHT) ? spec->tireMuLatFront : spec->tireMuLatRear;
                const float nx = wheel->forceLongitudinalN /
                                 (spec->tireMuLongScale * sv2->muLongitudinal *
                                  derived->tireLoadSensitivityMuScale[i] * wheel->normalLoadN);
                const float ny = wheel->forceLateralN /
                                 (lateralMuAxle2 * (sv2->muLateral / SURFACE_REFERENCE_MU_LAT) *
                                  derived->tireLoadSensitivityMuScale[i] * wheel->normalLoadN);
                wheel->frictionUsage = fminf(sqrtf(nx * nx + ny * ny), 1.0f);
            }
        }
    }

    derived->frontLateralForceN = 0.0f;
    derived->rearLateralForceN = 0.0f;
    derived->frontBodyForceN = (Vector2){ 0.0f, 0.0f };
    derived->rearBodyForceN = (Vector2){ 0.0f, 0.0f };
    derived->totalYawTorqueNm = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const bool front = (i <= WHEEL_FRONT_RIGHT);
        /* Wheel-frame lateral force sum (for telemetry; before rotation). */
        if (front)
            derived->frontLateralForceN += state->wheels[i].forceLateralN;
        else
            derived->rearLateralForceN += state->wheels[i].forceLateralN;

        /* Rotate to body frame. Each front wheel uses its own steer angle (still equal to
         * frontRoadWheelAngleRad until Ackermann lands, but read from the per-wheel field). */
        const Vector2 bodyForceN = front ? physics_rotate_wheel_force_to_body(
                                               (Vector2){ state->wheels[i].forceLongitudinalN,
                                                          state->wheels[i].forceLateralN },
                                               state->wheels[i].steerAngleRad)
                                         : (Vector2){ state->wheels[i].forceLongitudinalN,
                                                      state->wheels[i].forceLateralN };

        if (front) {
            derived->frontBodyForceN.x += bodyForceN.x;
            derived->frontBodyForceN.y += bodyForceN.y;
        } else {
            derived->rearBodyForceN.x += bodyForceN.x;
            derived->rearBodyForceN.y += bodyForceN.y;
        }

        /* Four-wheel yaw torque: M_z = Σ (px.Fy_body − py.Fx_body). The px.Fy_body term
         * reproduces the existing axle yaw moment (front: +l_f.Fy, rear: −l_r.Fy); the
         * −py.Fx_body term captures the longitudinal force's lever arm through the track
         * width, which the centreline bicycle model omits. */
        derived->totalYawTorqueNm += state->wheels[i].localPositionM.x * bodyForceN.y -
                                     state->wheels[i].localPositionM.y * bodyForceN.x;
    }

    /* --- 13/14/15. sum tire forces, then add the two resistance forces --- */

    derived->totalBodyForceN =
        (Vector2){ derived->frontBodyForceN.x + derived->rearBodyForceN.x + resistanceBodyN.x,
                   derived->frontBodyForceN.y + derived->rearBodyForceN.y + resistanceBodyN.y };

    const VehicleDerivatives dynamic =
        dynamic_derivatives(spec, state, derived->totalBodyForceN, derived->totalYawTorqueNm);
    /* At kinematic speeds, steering geometry owns lateral/yaw motion. Longitudinal
     * acceleration still comes only from limited tire Fx plus the two resistance forces,
     * so steering a stationary car cannot create propulsion from a rotated lateral force. */
    const float kinematicLongitudinalForceN =
        cosf(state->frontRoadWheelAngleRad) *
            (state->wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
             state->wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN) +
        state->wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
        state->wheels[WHEEL_REAR_RIGHT].forceLongitudinalN + longitudinalResistanceN;
    const VehicleDerivatives kinematic =
        kinematic_derivatives(spec, state, kinematicLongitudinalForceN);
    derived->lowSpeedBlend = physics_low_speed_blend(state->velocityLongitudinalMps);
    const VehicleDerivatives solved =
        derivatives_lerp(kinematic, dynamic, derived->lowSpeedBlend);

    state->velocityLongitudinalMps += solved.velocityLongitudinalDerivativeMps2 * dt;
    state->velocityLateralMps += solved.velocityLateralDerivativeMps2 * dt;
    state->yawRateRadS += solved.yawRateDerivativeRadS2 * dt;
    state->headingRad = wrap_angle(state->headingRad + state->yawRateRadS * dt);

    const float c = cosf(state->headingRad);
    const float s = sinf(state->headingRad);
    const float worldVxMps = state->velocityLongitudinalMps * c - state->velocityLateralMps * s;
    const float worldVyMps = state->velocityLongitudinalMps * s + state->velocityLateralMps * c;
    state->positionM.x += worldVxMps * dt;
    state->positionM.y += worldVyMps * dt;

    /* --- 17b. braking-reversal guard ------------------------------------------------ */
    /* Braking cannot propel the car, only slow it. With the +35% longitudinal grip of
     * the Phase 4 surface correction (asphalt muLongitudinal == 1.35), the tire
     * reaction torque on locked wheels can briefly exceed the brake-torque capacity,
     * causing the wheel integrator to un-lock the wheel to rolling speed. That
     * eliminates braking force from the tire; when the wheel lands fractionally above
     * rolling speed on the following tick, the residual positive slip produces a small
     * propelling impulse - a 0.007 m/s speed increase in the launch-stop scenario.
     *
     * The guard below is a physical invariant, not a hack: braking torque can only
     * oppose the direction of travel, never reverse it, and never increase the speed
     * in the direction of travel. At the zero boundary it brings the body to rest
     * rather than crossing through. */
    if ((input->brake > 0.0f || input->handbrake > 0.0f) && input->throttle <= 0.0f) {
        /* Reconstruct the velocity before integration so we can detect sign flips and
         * speed increases caused by the current tick's force solution. The derivative
         * is the solved (blended) value, exactly what was accumulated into the state. */
        const float vxPrev =
            state->velocityLongitudinalMps - solved.velocityLongitudinalDerivativeMps2 * dt;
        /* A braking-only input may not increase speed in the direction of travel. */
        if (vxPrev > 0.0f && state->velocityLongitudinalMps > vxPrev) {
            state->velocityLongitudinalMps = vxPrev;
        } else if (vxPrev < 0.0f && state->velocityLongitudinalMps < vxPrev) {
            state->velocityLongitudinalMps = vxPrev;
        }
        /* Braking can never push the body past rest: the wheels stop turning, lock,
         * and the body coasts to a stop via residual resistance. A sign reversal is a
         * numeric overshoot, not a physical event. */
        if (vxPrev * state->velocityLongitudinalMps < 0.0f) {
            state->velocityLongitudinalMps = 0.0f;
        }
    }

    /* Constant across the four wheels: the slip at which the longitudinal curve peaks. */
    const float peakSlip = tanf(DRIFTY_PI / (2.0f * spec->tireCLong)) / spec->tireBLong;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        /* Body integration precedes wheel integration, so the brake/free-roll crossing
         * guard uses the updated contact speed rather than lagging one fixed tick behind. */
        const float wheelVxMps = state->velocityLongitudinalMps;
        const float previousOmegaRadS = state->wheels[i].angularVelocityRadS;
        const float radiusM = vehicle_wheel_radius_m(spec, i);
        float nextOmegaRadS = drivetrain_integrate_wheel(
            previousOmegaRadS, wheelVxMps, torques.driveTorqueNm[i],
            torques.serviceBrakeTorqueNm[i], torques.handbrakeTorqueNm[i],
            state->wheels[i].forceLongitudinalN, radiusM, spec->wheelInertiaKgM2, dt,
            &state->wheels[i].locked);

        /* Stiff-limit equilibrium treatment for the unbraked wheel.
         *
         * The wheel-plus-tire subsystem's time constant is well under a millisecond -
         * far below the fixed step - so under any steady sub-peak torque the physical
         * wheel reaches its balance slip within the tick. The explicit integrator cannot:
         * it either overshoots the balance point (a tick-scale limit cycle between engine
         * braking and tire reaction) or, when the start-of-step slip was zero, produces no
         * torque at all and freezes the wheel while the body moves beneath it, which
         * returns as a phantom multi-kilonewton force one tick later.
         *
         * So when a stable equilibrium exists and the wheel is inside the pre-peak band
         * around it - the region where the linearized dynamics converge within one tick -
         * land on the equilibrium directly. Outside the band (deep wheelspin shedding
         * speed through a saturated curve) the multi-tick explicit dynamics are the right
         * answer and are kept. Torques at or beyond the curve's peak have no equilibrium
         * and always stay explicit; braking keeps its own lock/release dynamics. The
         * pure-curve balance ignores ellipse scaling, so under heavy combined slip the
         * landing is slightly conservative; that error is bounded by the ellipse, while
         * the artifacts this removes are an order of magnitude larger. */
        if (torques.serviceBrakeTorqueNm[i] <= 0.0f && torques.handbrakeTorqueNm[i] <= 0.0f) {
            float equilibriumOmegaRadS;
            if (drivetrain_wheel_equilibrium_omega(
                    torques.driveTorqueNm[i], wheelVxMps, radiusM,
                    spec->tireMuLongScale *
                        Surface_Get(state->wheels[i].surfaceId)->muLongitudinal *
                        derived->tireLoadSensitivityMuScale[i] * state->wheels[i].normalLoadN,
                    spec->tireBLong, spec->tireCLong, SLIP_SPEED_EPSILON_MPS,
                    &equilibriumOmegaRadS)) {
                const float vxSafeMps = fmaxf(fabsf(wheelVxMps), SLIP_SPEED_EPSILON_MPS);
                const float bandRadS = peakSlip * vxSafeMps / radiusM;
                const bool crossed = (previousOmegaRadS - equilibriumOmegaRadS) *
                                         (nextOmegaRadS - equilibriumOmegaRadS) <
                                     0.0f;
                if (crossed || fabsf(previousOmegaRadS - equilibriumOmegaRadS) < bandRadS) {
                    nextOmegaRadS = equilibriumOmegaRadS;
                    state->wheels[i].locked = false;
                }
            }
        }
        state->wheels[i].angularVelocityRadS = nextOmegaRadS;
    }
    /* Locked-axle enforcement: only for LOCKED mode, applied to each DRIVEN axle.
     * OPEN and LSD let wheels rotate independently, so no equalization is performed. */
    if (diffMode == DIFF_LOCKED && frontShare > 0.0f)
        locked_axle_equalize(state, WHEEL_FRONT_LEFT, WHEEL_FRONT_RIGHT);
    if (diffMode == DIFF_LOCKED && frontShare < 1.0f)
        locked_axle_equalize(state, WHEEL_REAR_LEFT, WHEEL_REAR_RIGHT);
    if (diffMode == DIFF_LOCKED && torques.totalGearRatio != 0.0f) {
        const float redlineWheelOmegaRadS =
            spec->engineRedlineRpm * DRIFTY_TWO_PI / (60.0f * fabsf(torques.totalGearRatio));
        if (frontShare > 0.0f)
            locked_axle_clamp_redline(state, WHEEL_FRONT_LEFT, WHEEL_FRONT_RIGHT,
                                      redlineWheelOmegaRadS);
        if (frontShare < 1.0f)
            locked_axle_clamp_redline(state, WHEEL_REAR_LEFT, WHEEL_REAR_RIGHT,
                                      redlineWheelOmegaRadS);
    }
    {
        float postOmega[WHEEL_COUNT];
        for (int i = 0; i < WHEEL_COUNT; i++)
            postOmega[i] = state->wheels[i].angularVelocityRadS;
        state->engineRpm = drivetrain_engine_rpm(
            spec, state->selectedGear, drivetrain_driven_mean_omega(postOmega, frontShare));
    }

    /* Body acceleration is force divided by mass. Do not reconstruct it from the integrated
     * derivative: dvx/dt also contains the rotating-frame transport term r*vy, and using the
     * post-integration state would add a small same-step error. The low-speed blend changes
     * state derivatives, not the force solution used by load transfer and telemetry. */
    derived->longitudinalAccelerationMps2 = derived->totalBodyForceN.x / spec->massKg;
    /* The kinematic low-speed model deliberately suppresses tire-model lateral force at
     * rest, so its reported lateral acceleration follows the solved blended derivative. */
    derived->lateralAccelerationMps2 = solved.velocityLateralDerivativeMps2 +
                                       state->yawRateRadS * state->velocityLongitudinalMps;

    /* --- 23. store the solved body-longitudinal acceleration for the next step ----------- */

    /* ax_body, not dvx_dt: load transfer responds to the longitudinal force on the body, and
     * dvx_dt carries the rotational transport term r*vy, which pitches nothing. */
    derived->solvedLongAccelMps2 = derived->longitudinalAccelerationMps2;
    state->prevLongAccelMps2 = derived->longitudinalAccelerationMps2;
    state->prevLatAccelMps2 = derived->totalBodyForceN.y / spec->massKg;
    derived->speedMps = sqrtf(state->velocityLongitudinalMps * state->velocityLongitudinalMps +
                              state->velocityLateralMps * state->velocityLateralMps);
    derived->bodySideslipRad =
        (derived->speedMps < LOW_SPEED_EPSILON_MPS)
            ? 0.0f
            : atan2f(state->velocityLateralMps,
                     fmaxf(fabsf(state->velocityLongitudinalMps), LOW_SPEED_EPSILON_MPS));
    derived->maxFrictionUsage = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        derived->maxFrictionUsage =
            fmaxf(derived->maxFrictionUsage, state->wheels[i].frictionUsage);
    }
    derived->physicallySliding = derived->maxFrictionUsage >= 0.98f;
    /* scoringDrift is owned by scoring_classify() in scoring.c - it is
     * overwritten in game_fixed_update() after physics returns. Keeping it false
     * here as the safe default for any path that skips classification. */
    derived->scoringDrift = false;

    renderState->currPositionM = state->positionM;
    renderState->currHeadingRad = state->headingRad;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        renderState->currWheelAngleRad[i] = state->wheels[i].steerAngleRad;
    }

    const bool valid = physics_state_is_valid(spec, state, derived);
    assert(valid);
    if (!valid) {
        *state = lastGoodState;
        *derived = lastGoodDerived;
        *renderState = lastGoodRenderState;
    }
}
