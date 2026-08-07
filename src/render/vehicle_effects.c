/*
 * vehicle_effects.c — the neutral placeholder for dynamic vehicle feedback.
 *
 * docs/DYNAMIC_CAR_VISUAL_EFFECTS_PROPOSAL.md defines the contract; the effect thresholds it
 * depends on are not agreed yet, and the taillight overlay additionally waits on the derived
 * lamp geometry from docs/CAR_VISUAL_PRIMITIVES_PROPOSAL.md. Until both land, this returns the
 * neutral state for every input, so a caller that wires the overlays draws exactly what it
 * draws today rather than failing to link against a declaration with no definition.
 */
#include "render/vehicle_effects.h"

#include <string.h>

VehicleVisualEffects vehicle_visual_effects_derive(const VehicleEffectInputs *inputs)
{
    /* Deliberately input-independent for now: see the file comment. The NULL case is part of
     * the published contract and stays true when the real derivation replaces this. */
    (void)inputs;

    VehicleVisualEffects out;
    memset(&out, 0, sizeof(out));
    return out;
}
