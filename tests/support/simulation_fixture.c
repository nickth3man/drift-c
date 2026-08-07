/*
 * simulation_fixture.c — the Game -> TelemetryRow projection.
 */
#include <string.h>
#include <math.h>

#include "simulation_fixture.h"

#include "core/config.h"
#include "physics/vehicle.h"

TelemetryRow test_telemetry_row_from_game(const Game *game, int substepCount)
{
    TelemetryRow row;
    memset(&row, 0, sizeof(row));
    row.tick = game->sim.tick;
    row.timeS = (double)game->sim.tick * (double)FIXED_DT_S;
    row.positionXM = game->vehicle.positionM.x;
    row.positionYM = game->vehicle.positionM.y;
    row.headingRad = game->vehicle.headingRad;
    row.velocityLongitudinalMps = game->vehicle.velocityLongitudinalMps;
    row.velocityLateralMps = game->vehicle.velocityLateralMps;
    row.speedMps = game->derived.speedMps;
    row.yawRateRadS = game->vehicle.yawRateRadS;
    row.steeringAngleRad = game->vehicle.frontRoadWheelAngleRad;
    row.engineRpm = game->vehicle.engineRpm;
    row.selectedGear = game->vehicle.selectedGear;
    row.frontSlipAngleRad = game->derived.frontSlipAngleRad;
    row.rearSlipAngleRad = game->derived.rearSlipAngleRad;
    row.frontSlipRatio = 0.5f * (game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio +
                                 game->vehicle.wheels[WHEEL_FRONT_RIGHT].slipRatio);
    row.rearSlipRatio = 0.5f * (game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio +
                                game->vehicle.wheels[WHEEL_REAR_RIGHT].slipRatio);
    row.frontWheelOmegaRadS =
        0.5f * (game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS +
                game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS);
    row.rearWheelOmegaRadS =
        0.5f * (game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS +
                game->vehicle.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS);
    row.frontNormalLoadN = game->derived.normalLoadFrontN;
    row.rearNormalLoadN = game->derived.normalLoadRearN;
    row.frontFxPureN = game->derived.pureLongitudinalForceN[WHEEL_FRONT_LEFT] +
                       game->derived.pureLongitudinalForceN[WHEEL_FRONT_RIGHT];
    row.rearFxPureN = game->derived.pureLongitudinalForceN[WHEEL_REAR_LEFT] +
                      game->derived.pureLongitudinalForceN[WHEEL_REAR_RIGHT];
    row.frontFyPureN = game->derived.pureLateralForceN[WHEEL_FRONT_LEFT] +
                       game->derived.pureLateralForceN[WHEEL_FRONT_RIGHT];
    row.rearFyPureN = game->derived.pureLateralForceN[WHEEL_REAR_LEFT] +
                      game->derived.pureLateralForceN[WHEEL_REAR_RIGHT];
    row.frontFxLimitedN = game->vehicle.wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
                          game->vehicle.wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN;
    row.rearFxLimitedN = game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                         game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN;
    row.frontFyLimitedN = game->derived.frontLateralForceN;
    row.rearFyLimitedN = game->derived.rearLateralForceN;
    row.frontFrictionUsage = fmaxf(game->vehicle.wheels[WHEEL_FRONT_LEFT].frictionUsage,
                                   game->vehicle.wheels[WHEEL_FRONT_RIGHT].frictionUsage);
    row.rearFrictionUsage = fmaxf(game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage,
                                  game->vehicle.wheels[WHEEL_REAR_RIGHT].frictionUsage);
    row.frontLocked = game->vehicle.wheels[WHEEL_FRONT_LEFT].locked ||
                      game->vehicle.wheels[WHEEL_FRONT_RIGHT].locked;
    row.rearLocked = game->vehicle.wheels[WHEEL_REAR_LEFT].locked ||
                     game->vehicle.wheels[WHEEL_REAR_RIGHT].locked;
    row.driveTorqueNm = game->derived.driveTorqueNm[WHEEL_FRONT_LEFT] +
                        game->derived.driveTorqueNm[WHEEL_FRONT_RIGHT] +
                        game->derived.driveTorqueNm[WHEEL_REAR_LEFT] +
                        game->derived.driveTorqueNm[WHEEL_REAR_RIGHT];
    row.frontBrakeTorqueNm = game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
                             game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT];
    row.rearBrakeTorqueNm = game->derived.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
                            game->derived.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT];
    row.handbrakeTorqueNm = game->derived.handbrakeTorqueNm[WHEEL_REAR_LEFT] +
                            game->derived.handbrakeTorqueNm[WHEEL_REAR_RIGHT];
    row.totalForceXN = game->derived.totalBodyForceN.x;
    row.totalForceYN = game->derived.totalBodyForceN.y;
    row.yawTorqueNm = game->derived.totalYawTorqueNm;
    row.bodySideslipRad = game->derived.bodySideslipRad;
    row.lowSpeedBlend = game->derived.lowSpeedBlend;
    row.substepCount = substepCount;
    row.backlogDrops = game->physicsBacklogDrops;
    row.stateChecksum = game->stateChecksum;

    row.staticFrontLoadN = game->derived.staticFrontLoadN;
    row.staticRearLoadN = game->derived.staticRearLoadN;
    row.dynamicFrontLoadN = game->derived.normalLoadFrontN;
    row.dynamicRearLoadN = game->derived.normalLoadRearN;
    row.loadTransferN = game->derived.loadTransferN;
    row.previousLongAccelMps2 = game->derived.previousLongAccelMps2;
    row.filteredLongAccelMps2 = game->derived.filteredLongAccelMps2;
    row.solvedLongAccelMps2 = game->derived.solvedLongAccelMps2;
    row.lateralAccelMps2 = game->derived.lateralAccelerationMps2;
    row.aeroDragN = game->derived.aeroDragMagnitudeN;
    row.aeroDragXN = game->derived.aeroDragBodyN.x;
    row.aeroDragYN = game->derived.aeroDragBodyN.y;
    row.rollingResistanceN = game->derived.rollingResistanceMagnitudeN;
    row.rollingResistanceXN = game->derived.rollingResistanceBodyN.x;
    row.rollingResistanceYN = game->derived.rollingResistanceBodyN.y;

    /* dev.appliedInput is the input the fixed update actually used, which is the scripted
     * timeline while a scenario runs rather than whatever game->input holds. */
    row.steeringInput = game->dev.appliedInput.steer;
    row.throttleInput = game->dev.appliedInput.throttle;
    row.brakeInput = game->dev.appliedInput.brake;
    row.handbrakeInput = game->dev.appliedInput.handbrake;
    row.surfaceFrontLeft = game->vehicle.wheels[WHEEL_FRONT_LEFT].surfaceId;
    row.surfaceFrontRight = game->vehicle.wheels[WHEEL_FRONT_RIGHT].surfaceId;
    row.surfaceRearLeft = game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId;
    row.surfaceRearRight = game->vehicle.wheels[WHEEL_REAR_RIGHT].surfaceId;
    return row;
}
