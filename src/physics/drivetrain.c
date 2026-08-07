#include "physics/drivetrain.h"

#include <math.h>
#include <string.h>

#include "core/math_utils.h"

static float signf_nonzero(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

float drivetrain_engine_torque_at_rpm(const VehicleSpec *spec, float engineRpm)
{
    if (spec == NULL ||
        !(isfinite(spec->engineIdleRpm) && isfinite(spec->engineRedlineRpm) &&
          spec->engineRedlineRpm > spec->engineIdleRpm) ||
        !isfinite(engineRpm))
        return 0.0f;

    const float clampedRpm = clampf(engineRpm, spec->engineIdleRpm, spec->engineRedlineRpm);
    const float position = (clampedRpm - spec->engineIdleRpm) /
                           (spec->engineRedlineRpm - spec->engineIdleRpm) *
                           (float)(ENGINE_CURVE_POINTS - 1);
    int lower = (int)floorf(position);
    if (lower < 0) lower = 0;
    if (lower >= ENGINE_CURVE_POINTS - 1) {
        return spec->engineTorqueCurveNm[ENGINE_CURVE_POINTS - 1];
    }
    return lerpf(spec->engineTorqueCurveNm[lower], spec->engineTorqueCurveNm[lower + 1],
                 position - (float)lower);
}

float drivetrain_total_gear_ratio(const VehicleSpec *spec, int selectedGear)
{
    if (spec == NULL || spec->gearCount <= 0 || spec->gearCount > MAX_GEARS ||
        !isfinite(spec->finalDriveRatio) || !(spec->finalDriveRatio > 0.0f))
        return 0.0f;
    if (selectedGear == 0) return 0.0f;
    if (selectedGear == -1) {
        if (!(isfinite(spec->reverseGearRatio) && spec->reverseGearRatio > 0.0f)) return 0.0f;
        return -spec->reverseGearRatio * spec->finalDriveRatio;
    }
    if (selectedGear < 1 || selectedGear > spec->gearCount) return 0.0f;
    return spec->gearRatios[selectedGear - 1] * spec->finalDriveRatio;
}

float drivetrain_engine_rpm(const VehicleSpec *spec, int selectedGear,
                            float drivenAngularVelocityRadS)
{
    if (spec == NULL || !isfinite(drivenAngularVelocityRadS)) return 0.0f;
    const float totalGear = drivetrain_total_gear_ratio(spec, selectedGear);
    if (totalGear == 0.0f) return spec->engineIdleRpm;
    const float rpm =
        fabsf(drivenAngularVelocityRadS) * fabsf(totalGear) * 60.0f / DRIFTY_TWO_PI;
    return clampf(rpm, spec->engineIdleRpm, spec->engineRedlineRpm);
}

float drivetrain_front_torque_share(const VehicleSpec *spec)
{
    if (spec == NULL) return 0.0f;
    switch ((int)spec->drivetrainLayout) {
        case 1: return 1.0f;                                       /* FWD */
        case 2: return clampf(spec->frontTorqueSplit, 0.0f, 1.0f); /* AWD */
        default: return 0.0f;                                      /* RWD */
    }
}

float drivetrain_driven_mean_omega(const float omegaRadS[WHEEL_COUNT], float frontShare)
{
    if (omegaRadS == NULL) return 0.0f;
    const float frontMean = 0.5f * (omegaRadS[WHEEL_FRONT_LEFT] + omegaRadS[WHEEL_FRONT_RIGHT]);
    const float rearMean = 0.5f * (omegaRadS[WHEEL_REAR_LEFT] + omegaRadS[WHEEL_REAR_RIGHT]);

    /* Exact equality rather than a tolerance: these are the endpoints the layout produces
     * verbatim, and taking the pure-axle branch keeps a RWD or FWD car's engine speed
     * bit-identical to the single-axle expression it had before AWD existed. */
    if (frontShare <= 0.0f) return rearMean;
    if (frontShare >= 1.0f) return frontMean;

    /* AWD: the centre differential ties both axles to one carrier, so the engine sees their
     * mean, not the driven-torque-weighted blend. A wheel that is spinning raises engine
     * speed whether or not its axle is receiving the larger share. */
    return 0.5f * (frontMean + rearMean);
}

void drivetrain_split_axle_torque(DifferentialMode mode, float axleTorqueNm,
                                  float omegaLeftRadS, float omegaRightRadS,
                                  float tireReactionTorqueLeftNm,
                                  float tireReactionTorqueRightNm, float biasRatio,
                                  float preloadNm, float *torqueLeftNm, float *torqueRightNm)
{
    /* Baseline: equal torque split (LOCKED and OPEN). */
    const float halfAxleTorqueNm = axleTorqueNm * 0.5f;
    float left = halfAxleTorqueNm;
    float right = halfAxleTorqueNm;

    if (mode == DIFF_LSD) {
        /* Torque-biasing clutch: the LSD transfers torque from the faster wheel to the
         * slower wheel, limited by the grip of the wheel with less traction. */
        const float dOmega = omegaLeftRadS - omegaRightRadS;
        if (fabsf(dOmega) > DIFF_OMEGA_EPSILON_RAD_S) {
            const float gripTorqueMin =
                fminf(fabsf(tireReactionTorqueLeftNm), fabsf(tireReactionTorqueRightNm));
            const float capacity = preloadNm + (biasRatio - 1.0f) * gripTorqueMin;
            const float dT = fminf(capacity, fabsf(axleTorqueNm * 0.5f));
            if (dOmega > 0.0f) { /* left faster → bias torque to right */
                left -= dT;
                right += dT;
            } else { /* right faster → bias torque to left */
                left += dT;
                right -= dT;
            }
        }
    }

    if (torqueLeftNm != NULL) *torqueLeftNm = left;
    if (torqueRightNm != NULL) *torqueRightNm = right;
}

DrivetrainTorques drivetrain_calculate_torques(const VehicleSpec *spec, int selectedGear,
                                               const float omegaRadS[WHEEL_COUNT],
                                               const float tireReactionTorqueNm[WHEEL_COUNT],
                                               float throttle, float brake, float handbrake)
{
    DrivetrainTorques out;
    memset(&out, 0, sizeof(out));
    if (spec == NULL || omegaRadS == NULL || tireReactionTorqueNm == NULL) return out;

    throttle = clampf(throttle, 0.0f, 1.0f);
    brake = clampf(brake, 0.0f, 1.0f);
    handbrake = clampf(handbrake, 0.0f, 1.0f);

    const DifferentialMode diffMode = (DifferentialMode)(int)spec->differentialMode;
    const float frontShare = drivetrain_front_torque_share(spec);

    /* Engine RPM is derived from the average DRIVEN omega for consistency across all diff
     * modes (the engine is connected to the differential, not individual wheels). */
    const float drivenAngularVelocityRadS = drivetrain_driven_mean_omega(omegaRadS, frontShare);
    out.totalGearRatio = drivetrain_total_gear_ratio(spec, selectedGear);
    const float rpm = drivetrain_engine_rpm(spec, selectedGear, drivenAngularVelocityRadS);
    float curveTorqueNm = drivetrain_engine_torque_at_rpm(spec, rpm);

    /* Rev limiter: drive torque tapers to zero over the last 500 rpm before redline. The
     * curve sampler clamps to its last point, and the curve's value AT redline is nonzero,
     * so without a limiter a wheel that breaks loose free-revs the carrier at full burnout
     * torque indefinitely - measured: rear omegas run to 944 rad/s (14x redline wheel
     * speed in gear 1) during a 4 s launch with an LSD, storing ~1 MJ of wheelspin that
     * the next grip recovery dumps into the body. The LOCKED-only omega clamp used to
     * mask this. A progressive taper, not a hard cut, so the carrier settles smoothly
     * against the limiter instead of banging full/zero torque each tick (the bang-bang
     * cycle oscillated longitudinal load transfer). Engine braking is unaffected: it is
     * subtracted separately below. */
    if (out.totalGearRatio != 0.0f) {
        const float rawRpm = fabsf(drivenAngularVelocityRadS) * fabsf(out.totalGearRatio) *
                             60.0f / DRIFTY_TWO_PI;
        const float limiterStartRpm = spec->engineRedlineRpm - 500.0f;
        if (rawRpm > limiterStartRpm) {
            const float fade = clampf((spec->engineRedlineRpm - rawRpm) / 500.0f, 0.0f, 1.0f);
            curveTorqueNm *= fade;
        }
    }

/* The simple engine model has no clutch state. Suppressing closed-throttle engine
     * braking while the axle is stationary prevents idle torque from launching the car
     * backwards; once rotating, the documented signed gear ratio makes it oppose motion.
     *
     * Engine braking also fades out as the driveline approaches idle speed. An engine
     * held at idle by its own fueling cannot keep extracting energy from the axle — in a
     * real car the clutch slips or the engine stalls — and without the fade the constant
     * torque drags the rear axle below its rolling speed every tick at walking pace,
     * exciting a two-tick limit cycle between engine braking and tire reaction. The fade
     * is linear from zero at idle to full at ENGINE_BRAKING_FADE_RPM_SPAN above it, using
     * the UNCLAMPED driveline rpm so the fade engages exactly where the idle clamp does. */
#define ENGINE_BRAKING_FADE_RPM_SPAN 300.0f
    float engineBrakingScale = 0.0f;
    if (fabsf(drivenAngularVelocityRadS) > 1e-4f && out.totalGearRatio != 0.0f) {
        const float rawRpm = fabsf(drivenAngularVelocityRadS) * fabsf(out.totalGearRatio) *
                             60.0f / DRIFTY_TWO_PI;
        engineBrakingScale =
            clampf((rawRpm - spec->engineIdleRpm) / ENGINE_BRAKING_FADE_RPM_SPAN, 0.0f, 1.0f);
    }
    const float engineBrakingNm =
        engineBrakingScale * (1.0f - throttle) * spec->engineBrakingTorqueNm;
    out.engineTorqueNm = throttle * curveTorqueNm - engineBrakingNm;
    out.drivelineTorqueNm =
        out.engineTorqueNm * out.totalGearRatio * spec->drivetrainEfficiency;

    /* Split the driveline torque across the axles, then through each driven axle's
     * differential. An axle receiving no share is left at zero rather than run through the
     * differential: multiplying by a zero share already gives zero torque, and skipping the
     * call keeps a single-axle car's arithmetic exactly what it was before AWD existed. */
    const float frontAxleTorqueNm = out.drivelineTorqueNm * frontShare;
    const float rearAxleTorqueNm = out.drivelineTorqueNm * (1.0f - frontShare);

    if (frontShare > 0.0f) {
        drivetrain_split_axle_torque(
            diffMode, frontAxleTorqueNm, omegaRadS[WHEEL_FRONT_LEFT],
            omegaRadS[WHEEL_FRONT_RIGHT], tireReactionTorqueNm[WHEEL_FRONT_LEFT],
            tireReactionTorqueNm[WHEEL_FRONT_RIGHT], spec->differentialBiasRatio,
            spec->differentialPreloadNm, &out.driveTorqueNm[WHEEL_FRONT_LEFT],
            &out.driveTorqueNm[WHEEL_FRONT_RIGHT]);
    }
    if (frontShare < 1.0f) {
        drivetrain_split_axle_torque(
            diffMode, rearAxleTorqueNm, omegaRadS[WHEEL_REAR_LEFT], omegaRadS[WHEEL_REAR_RIGHT],
            tireReactionTorqueNm[WHEEL_REAR_LEFT], tireReactionTorqueNm[WHEEL_REAR_RIGHT],
            spec->differentialBiasRatio, spec->differentialPreloadNm,
            &out.driveTorqueNm[WHEEL_REAR_LEFT], &out.driveTorqueNm[WHEEL_REAR_RIGHT]);
    }

    const float frontServiceTotalNm = brake * spec->maxBrakeTorqueNm * spec->brakeBiasFront;
    const float rearServiceTotalNm =
        brake * spec->maxBrakeTorqueNm * (1.0f - spec->brakeBiasFront);
    out.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] = frontServiceTotalNm * 0.5f;
    out.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT] = frontServiceTotalNm * 0.5f;
    out.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] = rearServiceTotalNm * 0.5f;
    out.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT] = rearServiceTotalNm * 0.5f;
    out.handbrakeTorqueNm[WHEEL_REAR_LEFT] = handbrake * spec->handbrakeTorqueNm * 0.5f;
    out.handbrakeTorqueNm[WHEEL_REAR_RIGHT] = handbrake * spec->handbrakeTorqueNm * 0.5f;
    return out;
}

float drivetrain_integrate_wheel(float angularVelocityRadS, float wheelLongitudinalVelocityMps,
                                 float driveTorqueNm, float serviceBrakeTorqueNm,
                                 float handbrakeTorqueNm, float tireLongitudinalForceN,
                                 float wheelRadiusM, float wheelInertiaKgM2, float dt,
                                 bool *locked)
{
    if (locked != NULL) *locked = false;
    if (!(isfinite(angularVelocityRadS) && isfinite(wheelLongitudinalVelocityMps) &&
          isfinite(driveTorqueNm) && isfinite(serviceBrakeTorqueNm) &&
          isfinite(handbrakeTorqueNm) && isfinite(tireLongitudinalForceN) &&
          isfinite(wheelRadiusM) && wheelRadiusM > 0.0f && isfinite(wheelInertiaKgM2) &&
          wheelInertiaKgM2 > 0.0f && isfinite(dt) && dt > 0.0f))
        return 0.0f;

    const float brakeMagnitudeNm =
        fmaxf(serviceBrakeTorqueNm, 0.0f) + fmaxf(handbrakeTorqueNm, 0.0f);
    if (brakeMagnitudeNm > 0.0f && fabsf(angularVelocityRadS) <= 1e-4f &&
        fabsf(wheelLongitudinalVelocityMps) <= 1e-4f && fabsf(driveTorqueNm) <= 1e-4f &&
        fabsf(tireLongitudinalForceN) <= 1e-4f) {
        if (locked != NULL) *locked = true;
        return 0.0f;
    }
    float brakeDirection = signf_nonzero(angularVelocityRadS);
    if (brakeDirection == 0.0f) brakeDirection = signf_nonzero(wheelLongitudinalVelocityMps);

    const float nonBrakeTorqueNm = driveTorqueNm - tireLongitudinalForceN * wheelRadiusM;
    const float netTorqueNm = nonBrakeTorqueNm - brakeMagnitudeNm * brakeDirection;
    float next = angularVelocityRadS + netTorqueNm / wheelInertiaKgM2 * dt;

    if (brakeMagnitudeNm > 0.0f && brakeDirection != 0.0f &&
        driveTorqueNm * brakeDirection <= brakeMagnitudeNm &&
        ((angularVelocityRadS * next < 0.0f) ||
         (fabsf(angularVelocityRadS) <= 1e-4f && next * brakeDirection <= 0.0f))) {
        next = 0.0f;
        if (locked != NULL) *locked = true;
    }
    if (brakeMagnitudeNm > 0.0f && brakeDirection != 0.0f) {
        const float rollingOmega = wheelLongitudinalVelocityMps / wheelRadiusM;
        if ((angularVelocityRadS - rollingOmega) * brakeDirection <= 0.0f &&
            (next - rollingOmega) * brakeDirection > 0.0f) {
            next = rollingOmega;
        }
    }
    if (brakeMagnitudeNm == 0.0f && driveTorqueNm * angularVelocityRadS < 0.0f &&
        angularVelocityRadS * next < 0.0f) {
        next = 0.0f;
    }
    if (driveTorqueNm != 0.0f && brakeMagnitudeNm == 0.0f) {
        const float rollingOmega = wheelLongitudinalVelocityMps / wheelRadiusM;
        const float driveDirection = signf_nonzero(driveTorqueNm);
        if ((angularVelocityRadS - rollingOmega) * driveDirection >= 0.0f &&
            (next - rollingOmega) * driveDirection < 0.0f) {
            next = rollingOmega;
        }
    }

    /* The explicit wheel equation is stiff around free rolling. A non-driven, unbraked
     * wheel may settle exactly at the rolling speed, but tire reaction may not numerically
     * throw it past that equilibrium in one fixed step. */
    if (driveTorqueNm == 0.0f && brakeMagnitudeNm == 0.0f) {
        const float rollingOmega = wheelLongitudinalVelocityMps / wheelRadiusM;
        if ((angularVelocityRadS - rollingOmega) * (next - rollingOmega) < 0.0f) {
            next = rollingOmega;
        }
    }

    /*
     * One-sided bound: only DRIVE torque may spin a wheel faster than the road is turning it.
     *
     * The tire force is evaluated from the slip at the start of the step, but the body has
     * already integrated by the time the wheel does, so the contact-patch rolling speed has
     * moved underneath it. Near a stop that stale force is enormous relative to the wheel's
     * inertia — dFx/domega * R * dt / I is around fifty at 120 Hz — and it can throw a braked
     * wheel from just below rolling speed to well above it in a single tick. The wheel then
     * has positive slip it did not earn, the tire pushes forward, and the brakes accelerate
     * the car.
     *
     * The road can only ever spin a wheel TOWARD its rolling speed, so bounding the result
     * on that side alone costs nothing physical: braking may still drag a wheel below rolling
     * speed and lock it, and drive torque may still break it loose into wheelspin.
     */
    const float rollingOmega = wheelLongitudinalVelocityMps / wheelRadiusM;
    const bool driveCanSpinUp =
        (rollingOmega >= 0.0f) ? (driveTorqueNm > 0.0f) : (driveTorqueNm < 0.0f);
    if (!driveCanSpinUp) {
        if (rollingOmega >= 0.0f) {
            next = fminf(next, fmaxf(angularVelocityRadS, rollingOmega));
        } else {
            next = fmaxf(next, fminf(angularVelocityRadS, rollingOmega));
        }
    }
    return isfinite(next) ? next : 0.0f;
}

bool drivetrain_wheel_equilibrium_omega(float driveTorqueNm, float wheelLongitudinalVelocityMps,
                                        float wheelRadiusM, float longitudinalLimitN,
                                        float tireBLong, float tireCLong,
                                        float slipSpeedEpsilonMps, float *equilibriumOmegaRadS)
{
    if (equilibriumOmegaRadS != NULL) *equilibriumOmegaRadS = 0.0f;
    if (!(isfinite(driveTorqueNm) && isfinite(wheelLongitudinalVelocityMps) &&
          isfinite(wheelRadiusM) && wheelRadiusM > 0.0f && isfinite(longitudinalLimitN) &&
          longitudinalLimitN > 0.0f && isfinite(tireBLong) && tireBLong > 0.0f &&
          isfinite(tireCLong) && tireCLong > 0.0f && isfinite(slipSpeedEpsilonMps) &&
          slipSpeedEpsilonMps > 0.0f))
        return false;

    const float normalized = driveTorqueNm / (wheelRadiusM * longitudinalLimitN);
    /* At or beyond the curve's peak there is no stable balance point: the reaction falls as
     * slip grows and the wheel genuinely runs away. Leave that to the explicit dynamics. */
    if (!(fabsf(normalized) < 0.999f)) return false;

    const float slipEq = tanf(asinf(normalized) / tireCLong) / tireBLong;
    const float vxSafe = fmaxf(fabsf(wheelLongitudinalVelocityMps), slipSpeedEpsilonMps);
    const float omegaEq = (wheelLongitudinalVelocityMps + slipEq * vxSafe) / wheelRadiusM;
    if (!isfinite(omegaEq)) return false;

    if (equilibriumOmegaRadS != NULL) *equilibriumOmegaRadS = omegaEq;
    return true;
}
