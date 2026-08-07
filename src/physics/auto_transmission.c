#include "physics/auto_transmission.h"
#include "physics/vehicle.h"
#include "game/input.h"

#define AUTO_STOP_THRESHOLD_MPS 0.5f
#define AUTO_NEUTRAL_DELAY_S 0.15f
#define AUTO_SHIFT_UP_FACTOR 0.85f
#define AUTO_SHIFT_DOWN_FACTOR 0.35f

void auto_transmission_update(AutoTransmission *at, VehicleState *vs, const VehicleSpec *spec,
                              const VehicleDerived *derived, Input *io, float dt)
{
    if (!at->enabled) return;

    switch (at->driveState) {
        case AUTO_DRIVE: {
            /* Auto-shift forward gears */
            const float upRpm = spec->engineRedlineRpm * AUTO_SHIFT_UP_FACTOR;
            const float downRpm = spec->engineRedlineRpm * AUTO_SHIFT_DOWN_FACTOR;

            if (vs->engineRpm > upRpm && vs->selectedGear < spec->gearCount)
                vs->selectedGear++;
            else if (vs->engineRpm < downRpm && vs->selectedGear > 1)
                vs->selectedGear--;

            /* Brake-to-stop → neutral → reverse */
            if (derived->speedMps < AUTO_STOP_THRESHOLD_MPS && io->brake > 0.0f) {
                vs->selectedGear = 0;
                at->driveState = AUTO_NEUTRAL;
                at->neutralTimer = 0.0f;
            }
            break;
        }

        case AUTO_REVERSE: {
            /* Check brake-to-stop BEFORE swapping (orig throttle = up = brake in reverse) */
            const float origThrottle = io->throttle;
            const float origBrake = io->brake;
            const bool shouldStop =
                (derived->speedMps < AUTO_STOP_THRESHOLD_MPS && origThrottle > 0.0f);

            /* Swap: down=throttle, up=brake */
            io->throttle = origBrake;
            io->brake = origThrottle;

            if (shouldStop) {
                vs->selectedGear = 0;
                at->driveState = AUTO_NEUTRAL;
                at->neutralTimer = 0.0f;
            }
            break;
        }

        case AUTO_NEUTRAL: {
            at->neutralTimer += dt;

            if (at->neutralTimer > AUTO_NEUTRAL_DELAY_S) {
                if (io->throttle > 0.0f) {
                    vs->selectedGear = 1;
                    at->driveState = AUTO_DRIVE;
                } else if (io->brake > 0.0f && !at->forwardOnly) {
                    vs->selectedGear = -1;
                    at->driveState = AUTO_REVERSE;
                }
            }

            /* Zero throttle AFTER the transition check. Forward-only mode keeps a stopped
             * research maneuver neutral while the service brake remains held. */
            io->throttle = 0.0f;
            break;
        }
    }
}
