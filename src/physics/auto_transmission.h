/*
 * auto_transmission.h — automatic gear selection and arcade direction-swap state machine.
 */
#ifndef DRIFTY_AUTO_TRANSMISSION_H
#define DRIFTY_AUTO_TRANSMISSION_H

#include <stdbool.h>

#include "physics/vehicle.h"
#include "game/input.h"

typedef enum { AUTO_DRIVE = 0, AUTO_NEUTRAL = 1, AUTO_REVERSE = 2 } AutoDriveState;

typedef struct {
    bool enabled;
    bool forwardOnly;
    AutoDriveState driveState;
    float neutralTimer;
} AutoTransmission;

void auto_transmission_update(AutoTransmission *at, VehicleState *vs, const VehicleSpec *spec,
                              const VehicleDerived *derived, Input *io, float dt);

#endif
