/*
 * car_appearance.h — stable, presentation-only identity for procedural vehicle art.
 *
 * This draft type deliberately lives outside VehicleSpec: paint and decorative identity are
 * not physics inputs and must not affect simulation validation, checksums, replays, or solver
 * behavior. A zero seed is valid, so `hasSeed` is the explicit presence bit.
 */
#ifndef DRIFTY_CAR_APPEARANCE_H
#define DRIFTY_CAR_APPEARANCE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool hasSeed;
    uint32_t seed;
} CarAppearanceSpec;

#endif /* DRIFTY_CAR_APPEARANCE_H */
