/*
 * car_roster.h — the six cars Milestone 1 validates, as pure functions of an index.
 *
 * WHY SIX, AND WHY THESE. Two per drivetrain layout, so that any claim about RWD, FWD or AWD
 * rests on more than one data point, and so that a difference between the two members of a
 * pair can only come from the things that actually differ between them. The roster exists to
 * make the validation suite argue with evidence: a lap time is only interesting next to
 * another lap time from a car that differs in a named way.
 *
 * WHY C AND NOT FILES, the same reason as car_corpus.h: the roster is the source of truth for
 * the suite, so it has to be reachable with no file I/O, no working-directory assumption, and
 * no directory enumeration from inside the hot-reloadable module. `drifty_tests
 * --generate-roster` exports each spec to data/vehicles/roster/ as .txt files in the existing
 * format for humans to read and diff, and the `roster` scenario asserts the export round-trips
 * so the files cannot rot away from the code.
 *
 * SEEDED, NOT INVENTED. Each entry starts from one of the hand-tuned presets in dev_presets.c
 * and then overrides only what the roster's job requires — the drivetrain layout, and whatever
 * a car of that layout must have to be a fair example of it (a front-drive car carries its
 * mass over the driven axle and brakes forward, or it is not a front-drive car). Reusing the
 * presets keeps one set of researched numbers rather than a second, quietly diverging copy.
 *
 * These specs are NOT normalised to make any driver's job easier. A car that cannot be got
 * round the circuit is a result, not a bug in the roster.
 *
 * Raylib-free and I/O-free, like car_corpus.h.
 */
#ifndef DRIFTY_CAR_ROSTER_H
#define DRIFTY_CAR_ROSTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "physics/vehicle.h"

/* Total cars in the roster. Stable for a given build. */
int car_roster_count(void);

/*
 * Write car `index` into `out`. Pure: the same index always yields the same spec, whatever
 * order the calls come in. Returns false for an out-of-range index or a NULL out.
 *
 * Validity is asserted by the `roster` scenario rather than guaranteed here, which is how a
 * bad override gets caught loudly instead of producing a subtly broken car.
 */
bool car_roster_spec(int index, VehicleSpec *out);

/* Stable, filesystem-safe identifier, e.g. "rwd_grip". Written into the caller's buffer so
 * there is no shared static state to go stale across a reload. */
void car_roster_id(int index, char *buf, size_t cap);

/* Human-readable name for a HUD or a report, e.g. "RWD Grip". Points at module rodata: fine
 * for immediate printing, never to be stored in Game (see hotreload.h). */
const char *car_roster_display_name(int index);

/* One line naming what this car is for, e.g. "high grip, moderate power — the reference". */
const char *car_roster_describe(int index);

/* "RWD" / "FWD" / "AWD", derived from the built spec rather than from a second table, so it
 * cannot disagree with what the car actually does. */
const char *car_roster_layout_name(int index);

/* FNV-1a over the built spec's bytes. Two runs quoting the same hash drove the same car, even
 * if someone edited a preset in between without bumping anything. */
uint32_t car_roster_spec_hash(int index);

/* Index of the car with this id, or -1. Lets `--car rwd_grip` resolve without the caller
 * knowing the table order. */
int car_roster_find(const char *id);

#endif /* DRIFTY_CAR_ROSTER_H */
