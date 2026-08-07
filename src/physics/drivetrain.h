/*
 * drivetrain.h — pure Phase 2 engine, gearing, torque split, and wheel integration.
 */
#ifndef DRIFTY_DRIVETRAIN_H
#define DRIFTY_DRIVETRAIN_H

#include "physics/vehicle.h"

typedef struct {
    float engineTorqueNm;
    float totalGearRatio;
    float drivelineTorqueNm;
    float driveTorqueNm[WHEEL_COUNT];
    float serviceBrakeTorqueNm[WHEEL_COUNT];
    float handbrakeTorqueNm[WHEEL_COUNT];
} DrivetrainTorques;

float drivetrain_engine_torque_at_rpm(const VehicleSpec *spec, float engineRpm);
float drivetrain_total_gear_ratio(const VehicleSpec *spec, int selectedGear);

/*
 * Fraction of driveline torque sent to the FRONT axle: 0 for RWD, 1 for FWD, the configured
 * frontTorqueSplit (clamped to [0,1]) for AWD. The rear axle receives the remainder.
 *
 * This is the single place the layout enum turns into a number. Everything downstream —
 * torque split, driven-wheel set, engine speed reference — is expressed in terms of this
 * share, so a layout is never tested for by name anywhere else.
 */
float drivetrain_front_torque_share(const VehicleSpec *spec);

/*
 * Engine speed from the driven wheels. drivenAngularVelocityRadS is the mean angular
 * velocity of whichever wheels the layout actually drives (both axles under AWD), which is
 * what the engine is geared to.
 */
float drivetrain_engine_rpm(const VehicleSpec *spec, int selectedGear,
                            float drivenAngularVelocityRadS);

/*
 * Mean angular velocity of the driven wheels for `frontShare`. Pure helper so the caller and
 * the torque calculation cannot disagree about which wheels the engine is connected to.
 */
float drivetrain_driven_mean_omega(const float omegaRadS[WHEEL_COUNT], float frontShare);

/*
 * One driven axle's differential, from carrier torque to the two wheel torques.
 *
 * Shared by every driven axle: with one implementation an FWD car's front differential
 * behaves exactly like an RWD car's rear one, and an AWD car gets the configured behaviour at
 * both ends without a second copy of the LSD maths.
 *
 * axleTorqueNm is the torque arriving at the carrier. LOCKED and OPEN split it equally; LSD
 * additionally biases torque from the faster wheel to the slower one, limited by the grip of
 * whichever wheel has less traction. Scalar parameters rather than a VehicleSpec so the
 * function stays pure and directly testable, matching drivetrain_integrate_wheel() below.
 */
void drivetrain_split_axle_torque(DifferentialMode mode, float axleTorqueNm,
                                  float omegaLeftRadS, float omegaRightRadS,
                                  float tireReactionTorqueLeftNm,
                                  float tireReactionTorqueRightNm, float biasRatio,
                                  float preloadNm, float *torqueLeftNm, float *torqueRightNm);

/*
 * Engine, gearing, torque split across the driven axle(s), and the brake torques.
 *
 * omegaRadS and tireReactionTorqueNm are indexed by WheelId and describe all four wheels; the
 * layout decides which of them the driveline is connected to. Tire reaction torque is the
 * torque the road exerts on a wheel (Fx * R) and feeds the LSD's grip limit — on the first
 * step after a reset these are 0, which conservatively limits an LSD to its preload.
 */
DrivetrainTorques drivetrain_calculate_torques(const VehicleSpec *spec, int selectedGear,
                                               const float omegaRadS[WHEEL_COUNT],
                                               const float tireReactionTorqueNm[WHEEL_COUNT],
                                               float throttle, float brake, float handbrake);

float drivetrain_integrate_wheel(float angularVelocityRadS, float wheelLongitudinalVelocityMps,
                                 float driveTorqueNm, float serviceBrakeTorqueNm,
                                 float handbrakeTorqueNm, float tireLongitudinalForceN,
                                 float wheelRadiusM, float wheelInertiaKgM2, float dt,
                                 bool *locked);

/*
 * The wheel speed at which the pure longitudinal tire reaction exactly balances a steady
 * drive torque — the implicit solution of the stiff wheel equation. Returns false when no
 * stable equilibrium exists, i.e. the torque demands a reaction at or beyond the curve's
 * peak (genuine wheelspin or lockup, where the explicit dynamics are the right answer).
 *
 * Inverting force = longLimitN * sin(C * atan(B * k)) for the slip k that reacts
 * driveTorqueNm / wheelRadiusM:
 *
 *     k_eq      = tan(asin(normalized) / C) / B
 *     omega_eq  = (wheel_vx + k_eq * max(|wheel_vx|, slipSpeedEpsilonMps)) / R
 */
bool drivetrain_wheel_equilibrium_omega(float driveTorqueNm, float wheelLongitudinalVelocityMps,
                                        float wheelRadiusM, float longitudinalLimitN,
                                        float tireBLong, float tireCLong,
                                        float slipSpeedEpsilonMps, float *equilibriumOmegaRadS);

#endif /* DRIFTY_DRIVETRAIN_H */
