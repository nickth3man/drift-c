/*
 * car_roster.c — the six cars Milestone 1 validates. See the header for the contract.
 *
 * Each entry starts from a dev_presets.c archetype (the researched baseline numbers) and
 * overrides only what its layout demands: the drivetrain configuration, and whatever a car
 * of that layout must carry to be a fair example of it (a front-drive car carries its mass
 * over the driven axle; an all-wheel-drive car publishes a front torque split). Reusing the
 * presets keeps one set of researched numbers rather than a second, quietly diverging copy.
 *
 * The roster is pure: the same index always yields the same spec, whatever order calls come
 * in. Raylib-free and I/O-free, like car_corpus.c.
 */
#include "game/car_roster.h"

#include <stdio.h>
#include <string.h>

#include "dev/dev_params.h"
#include "dev/dev_presets.h"
#include "physics/vehicle.h"

/* -------------------------------------------------------------------------------------------- layout --
 * The parameter keys that turn a preset into a specific layout. Each key is resolved through
 * the registry at apply time, so a typo is caught as an unknown key rather than a silent
 * offset mismatch. */
#define KEY_LAYOUT "drive.layout"
#define KEY_SPLIT "drive.front_torque_split"
#define KEY_CG_FRONT "body.cg_to_front"
#define KEY_CG_REAR "body.cg_to_rear"
#define KEY_BRAKE_BIAS "brake.bias_front"

/* One roster entry's configuration: which preset to seed from, plus the overrides that make
 * it the car the roster wants it to be. */
typedef struct {
    int presetIndex;        /* dev_presets.c index to seed from */
    int layout;             /* 0=RWD, 1=FWD, 2=AWD */
    float frontTorqueSplit; /* AWD only; ignored for RWD/FWD */
    const char *id;
    const char *displayName;
    const char *describe;
    /* Layout-specific overrides beyond the drivetrain. A FWD car carries its mass over the
     * driven axle, so cg_to_front/cg_to_rear and brake_bias_front are nudged. NULL-terminated
     * pairs; the sentinel is { NULL, 0 }. */
    const struct {
        const char *key;
        float value;
    } overrides[8];
} RosterEntry;

static const RosterEntry kRoster[] = {
    /* 0: rwd_grip — the reference car. Track Predator as-is; RWD is its native layout. */
    {
        4,
        0,
        0.0f,
        "rwd_grip",
        "RWD Grip",
        "high grip, moderate power, LSD \xe2\x80\x94 the reference",
        { { NULL, 0 } },
    },
    /* 1: rwd_power — Pro D1GP, high power, lower rear mu. RWD is native. */
    {
        2,
        0,
        0.0f,
        "rwd_power",
        "RWD Power",
        "high power, lower rear \xce\xbc \xe2\x80\x94 power oversteer available",
        { { NULL, 0 } },
    },
    /* 2: fwd_light — Shoebox converted to FWD. A front-drive kei car carries its mass over
     * the driven axle and brakes forward. */
    {
        8,
        1,
        0.0f,
        "fwd_light",
        "FWD Light",
        "light, low power \xe2\x80\x94 understeer-limited",
        {
            { KEY_CG_FRONT, 0.99f }, /* mass over driven axle */
            { KEY_CG_REAR, 1.21f },
            { KEY_BRAKE_BIAS, 0.62f },
            { NULL, 0 },
        },
    },
    /* 3: fwd_hot — a more powerful FWD car, seeded from Track Predator mass/power but with
     * front-drive layout and a forward weight bias. Torque steer and lift-off rotation. */
    {
        4,
        1,
        0.0f,
        "fwd_hot",
        "FWD Hot",
        "more power \xe2\x80\x94 torque steer, lift-off rotation",
        {
            { KEY_CG_FRONT, 1.10f },
            { KEY_CG_REAR, 1.45f },
            { KEY_BRAKE_BIAS, 0.60f },
            { NULL, 0 },
        },
    },
    /* 4: awd_rally — Rally Devil with AWD. 50/50 split, loose-surface bias. */
    {
        5,
        2,
        0.50f,
        "awd_rally",
        "AWD Rally",
        "50/50 split, loose-surface bias",
        { { NULL, 0 } },
    },
    /* 5: awd_gt — Track Predator converted to AWD. Front-biased split, high grip. */
    {
        4,
        2,
        0.40f,
        "awd_gt",
        "AWD GT",
        "front-biased split, high grip",
        { { NULL, 0 } },
    },
};

#define ROSTER_COUNT ((int)(sizeof(kRoster) / sizeof(kRoster[0])))

int car_roster_count(void)
{
    return ROSTER_COUNT;
}

bool car_roster_spec(int index, VehicleSpec *out)
{
    if (out == NULL || index < 0 || index >= ROSTER_COUNT) return false;

    const RosterEntry *entry = &kRoster[index];

    /* Start from the preset's researched baseline numbers. */
    dev_preset_apply(out, entry->presetIndex);

    /* Apply layout-specific overrides first, then the drivetrain configuration. The
     * registry resolves keys to offsets, so a typo becomes an unknown key, not a bad write. */
    DevParamAssignment items[16];
    int count = 0;

    for (int i = 0; entry->overrides[i].key != NULL && count < 16; i++) {
        items[count].key = entry->overrides[i].key;
        items[count].value = entry->overrides[i].value;
        count++;
    }

    items[count].key = KEY_LAYOUT;
    items[count].value = (float)entry->layout;
    count++;

    if (entry->layout == 2) {
        items[count].key = KEY_SPLIT;
        items[count].value = entry->frontTorqueSplit;
        count++;
    }

    dev_params_apply_assignments(out, items, count, NULL, NULL);
    return true;
}

void car_roster_id(int index, char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return;
    if (index < 0 || index >= ROSTER_COUNT) {
        buf[0] = '\0';
        return;
    }
    snprintf(buf, cap, "%s", kRoster[index].id);
}

const char *car_roster_display_name(int index)
{
    if (index < 0 || index >= ROSTER_COUNT) return "?";
    return kRoster[index].displayName;
}

const char *car_roster_describe(int index)
{
    if (index < 0 || index >= ROSTER_COUNT) return "";
    return kRoster[index].describe;
}

const char *car_roster_layout_name(int index)
{
    if (index < 0 || index >= ROSTER_COUNT) return "?";
    switch (kRoster[index].layout) {
        case 1: return "FWD";
        case 2: return "AWD";
        default: return "RWD";
    }
}

uint32_t car_roster_spec_hash(int index)
{
    VehicleSpec spec;
    if (!car_roster_spec(index, &spec)) return 0;

    /* FNV-1a over the spec bytes. Padding is zeroed by the memset inside vehicle_spec_set_default
     * (called through dev_preset_apply), so two specs with the same fields hash identically. */
    const unsigned char *bytes = (const unsigned char *)&spec;
    uint32_t hash = 0x811c9dc5u;
    for (size_t i = 0; i < sizeof(spec); i++) {
        hash ^= bytes[i];
        hash *= 0x01000193u;
    }
    return hash;
}

int car_roster_find(const char *id)
{
    if (id == NULL) return -1;
    for (int i = 0; i < ROSTER_COUNT; i++) {
        if (strcmp(kRoster[i].id, id) == 0) return i;
    }
    return -1;
}
