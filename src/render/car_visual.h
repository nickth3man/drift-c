/*
 * car_visual.h — the vehicle appearance grammar: VehicleSpec -> CarVisual.
 *
 * THE ONE RULE. A car's appearance is a pure, total, deterministic function of its physics
 * parameters. This translation unit is the only place in the project where a styling decision
 * may live. src/render/render.c and the contact-sheet writer are dumb consumers of CarVisual; neither
 * may invent geometry of its own.
 *
 * WHY IT LIVES HERE AND NOT IN render.c. render.c stubs its whole draw path out under
 * DRIFTY_HEADLESS (src/render/render.c), so anything decided there is unreachable from
 * drifty_tests.exe and therefore unverifiable. car_visual.c is raylib-free and sits in
 * SHARED_SRCS, so the grammar is linked into the headless test binary and is covered by the
 * `car-visual` and `corpus` scenarios.
 *
 * RAYLIB-FREE. This header includes raylib.h for the Vector2 and Color *types* only. Neither
 * it nor car_visual.c calls a raylib function, exactly like src/core/units.h. That is what keeps
 * drifty_tests linkable without a window.
 *
 * TAXONOMY OF A FEATURE. Every field below is produced by exactly one of these kinds of rule,
 * and car_visual.c labels each one:
 *
 *   [identity]  The drawn value IS the physical value. Wheel centres, wheelbase, tire
 *               diameter, body width. These may never be fudged: the `car-visual` scenario
 *               asserts the wheel centres equal vehicle.c's set_wheel_positions() exactly.
 *   [rule]      A documented, deterministic styling mapping from named parameters — e.g. the
 *               greenhouse sits where the CG bias puts it. Not a law of physics, but it cites
 *               its inputs and is stable.
 *
 * NO BYTE HASHING FOR GEOMETRY. car_visual.c must not derive any geometric feature from a hash
 * of the spec. Hashing would trivially satisfy the distinctness test while destroying the
 * property that test exists to protect. Colour is the single, stated exception: it is
 * explicitly arbitrary (see car_visual_colour_seed) and is excluded from the signature.
 * Livery stripe COLOUR may also use the colour-seed bits; stripe GEOMETRY (placement, span,
 * pattern) is a deterministic function of stripeWeight and body extents. Both rules are
 * documented inline.
 *
 * UNITS AND FRAME. Everything is in metres in the body frame: +X forward (nose), +Y left,
 * origin at the CG — the convention in src/core/units.h. Consumers apply their own
 * metres-to-pixels scale.
 *
 * PRESENTATION GAINS. Several real-world quantities are well below the ~7.6 cm / 1 px
 * visibility floor at the canonical PIXELS_PER_METER * CAMERA_BASE_ZOOM scale (~13.2 px/m).
 * They receive documented render-only amplification so they are visible:
 *
 *   CV_TOE_VISUAL_GAIN     static toe angle amplification (raw ~0.15° = 0.4 px,
 *                          gained to ~1.2° → perceptible steering-geometry cue).
 *   CV_CAMBER_VISUAL_GAIN  camber footprint-narrowing amplification (raw ~3 cm = 0.4 px,
 *                          gained to ~12 cm apparent reduction, visible as stance).
 *   CV_REST_ANGLE_GAIN     legacy Ackermann rest-angle amplification, 6.0×.
 *
 * These are applied ONLY in car_visual.c. No simulation quantity reads them.
 */
#ifndef DRIFTY_CAR_VISUAL_H
#define DRIFTY_CAR_VISUAL_H

#include <stdbool.h>
#include <stdint.h>

#include "raylib.h" /* Vector2 and Color types only; no raylib call is made */

#include "physics/vehicle.h"

/* ------------------------------------------------------------------ presentation gains --
 *
 * Render-only amplification for quantities below the one-pixel visibility floor. Every
 * constant below includes a comment stating WHY it exists: the raw effect, its size in pixels
 * at ~13.2 px/m, and the gained apparent size. These are not physics — no solver reads them.
 */
#define CV_TOE_VISUAL_GAIN \
    8.0f /* raw static toe ~0.15° → 0.4 px swing at tire edge;
                                         gained to ~1.2° → ~3 px, barely legible */
#define CV_CAMBER_VISUAL_GAIN \
    4.0f /* raw camber ~1.5° → 0.38 px footprint narrowing;
                                         gained cos distortion → ~1.5 px, visible stance */
#define CV_REST_ANGLE_GAIN \
    6.0f /* legacy: pre-Phase-2 Ackermann rest-angle amplification;
                                         kept for consistency until toe migration completes */
/* car_visual.c defines the remaining internal presentation constants: exhaust and race-marker
 * visibility gains, the minimum cabin span and forward package bias, and sub-pixel hull-facing
 * corrections. Consumers receive only the resulting CarVisual dimensions. */

/* Longitudinal stations of the hull half-outline, tail (index 0) to nose (last). The outline
 * is mirrored across the centreline, so a car is always laterally symmetric. Nine stations is
 * enough to express a wedge, a brick, a teardrop and a hipped muscle car while staying legible
 * at the ~13 px/m the game actually draws at. */
#define CAR_HULL_STATIONS 9
#define CAR_SIDE_WINDOWS_MAX 6
#define CAR_ROOF_PANELS_MAX 2

typedef struct {
    float centreXM;
    float lengthM;
} CarVisualSegment;

typedef struct {
    float xM;         /* body-space X, +X forward */
    float halfWidthM; /* hull half-width at this station, always >= 0 */
} CarHullStation;

typedef struct {
    Vector2 centreM; /* [identity] equals VehicleState.wheels[i].localPositionM */
    float diameterM; /* [identity] 2 * wheelRadius[Front|Rear]M; outer tread edge */
    float widthM; /* [identity] tire section width in m; reads tireSectionWidthFrontMm etc. */
    float
        rimDiameterM; /* [identity] 2× rim radius; reads tireRimDiameter[Front|Rear]In × 0.0254 */
    float
        rimWidthM; /* [identity] rim barrel width; reads tireRimWidth[Front|Rear]In × 0.0254 */
    float sidewallHeightM; /* [identity] visible sidewall band: (diameterM - rimDiameterM)/2;
                                        reads tireAspect[Front|Rear]Pct */
    float discDiameterM;   /* [rule] brake disc; reads brakeDiscRadius[Front|Rear]M directly */
    float
        staticAngleRad; /* [rule] rest toe × CV_TOE_VISUAL_GAIN; reads suspToe[Front|Rear]Rad */
    float camberVisualCos; /* [rule] cos(clamp(suspCamber × CV_CAMBER_VISUAL_GAIN));
                                        scales drawn tire width for stance cue */
    float pokeM;           /* [identity] how far the tire's outer edge stands beyond the body
                                        edge: trackWidth[Front|Rear]M/2 + tire width/2 -
                                        widthOverallM/2. It does NOT read wheelOffsetEt*Mm:
                                        with track width primary, the ET offset is already
                                        folded into the track the hubs sit at, and reading it
                                        again here would double-count it. */
    float archGapM;        /* [identity] wheel-arch clearance; reads rideHeight[Front|Rear]M
                                        + suspTravel[Front|Rear]M */
    float archFlareM;      /* [rule] this arch's flare: the shared derived flare (track, tire
                                        width, arch gap, openWheelWeight) plus this axle's
                                        declared body.fender_flare_[front|rear]. The rasterizer
                                        reads THIS, never the shared value, so a front-only
                                        flare cannot widen the rear arch. */
} CarWheelVisual;

/* The normalised style axes. Derived first so that ~30 features move together instead of
 * independently: a heavy, tall, soft spec should read as van-like across silhouette, arches,
 * wing and exhaust at once. Exposed for test diagnostics. */
typedef struct {
    float mass01;    /* massKg */
    float size01;    /* wheelbase */
    float low01;     /* 1 - cgHeight; "how planted" */
    float grip01;    /* max lateral mu */
    float balance01; /* muFront - muRear; oversteer bias */
    float power01;   /* peak engine torque * final drive / mass */
    float aero01;    /* dragCoefficient * frontalArea */
    float sport01;   /* composite of grip/low/power */
    float strip01;   /* light-for-its-size => racecar */
} CarLatents;

typedef struct {
    /* ---- silhouette ---- */
    CarHullStation hull[CAR_HULL_STATIONS];
    float lengthM;        /* nose to tail */
    float widthM;         /* widest point, both sides */
    float wheelbaseM;     /* [identity] cgToFrontM + cgToRearM */
    float frontOverhangM; /* body ahead of the front axle */
    float rearOverhangM;  /* body behind the rear axle */
    float shoulderXM;     /* [identity] body-frame station of maximum width; reads shoulderXM
                                        via layout_to_body_x (layout frame -> body frame) */

    /* ---- greenhouse ---- */
    float cabinCentreXM;
    float cabinLengthM;
    float cabinHalfWidthM;
    float windscreenXM; /* forward foot of the glass */
    float backlightXM;  /* rear foot of the glass */
    float roofStartXM;  /* [identity] forward roof edge, converted from layout frame */
    float roofEndXM;    /* [identity] aft roof edge, converted from layout frame */
    float roofWidthM;   /* [identity] full physical roof width */
    float roofLengthM;  /* [identity] roofStartXM - roofEndXM */
    float glassHalfWidthM;
    float windscreenLengthM;        /* [rule] plan projection from windscreen rake */
    float backlightLengthM;         /* [rule] plan projection from backlight rake */
    float roofHighlightWidthM;      /* [rule] in-hull body-height cue within roof panels */
    float roofHighlightLengthScale; /* [rule] body-height cue, 0..1 of each panel */
    CarVisualSegment roofPanels[CAR_ROOF_PANELS_MAX];
    int roofPanelCount;
    CarVisualSegment sideWindows[CAR_SIDE_WINDOWS_MAX];
    int sideWindowCount;
    float sideWindowBandWidthM;
    CarVisualSegment quarterWindow;
    CarVisualSegment sunroof;
    int doorCount;
    int cabinRows;
    int roofType;
    /* ---- wheels ---- */
    CarWheelVisual wheels[WHEEL_COUNT];
    float archFlareM; /* [rule] shared derived flare, before the declared per-axle add */
    float
        fenderFlareFrontM; /* [identity] declared front arch flare; folded into wheels[].archFlareM */
    float
        fenderFlareRearM; /* [identity] declared rear arch flare; folded into wheels[].archFlareM */

    /* ---- appendages; a zero magnitude means absent ---- */
    float wingSpanM;  /* [rule] wing span; reads |aeroLiftCoefRear| × aeroRefAreaRearM2 */
    float wingChordM; /* [rule] wing chord from rear aero magnitude */
    float wingXM;     /* [rule] wing longitudinal position near tail */
    float
        splitterProtrusionM; /* [rule] splitter; reads |aeroLiftCoefFront| × aeroRefAreaFrontM2 */
    float splitterWidthM;    /* [rule] splitter width from front aero magnitude */
    float canardStrength;    /* [rule] 0..1; reads |aeroLiftCoefFront| × aeroRefAreaFrontM2 */
    float mirrorOffsetM;
    float exhaustBoreM;      /* [rule] exhaust bore; reads engineDisplacementL */
    int exhaustCount;        /* [rule] exhaust count 1..4; reads engineCylinders */
    float exhaustTransition; /* [rule] 0..1 within-tier fraction; scales pipe bore in the raster
                                       so count × bore² stays continuous across thresholds */
    float towHookDiameterM; /* [rule] presentation-gained race marker; reads raceDetailWeight */
    float towHookXM;        /* [rule] marker station clear of the L9 heading triangle */
    float hoodPinDiameterM; /* [rule] presentation-gained race marker; reads raceDetailWeight */
    float headingLengthM;   /* [rule] gameplay marker length; reads overall body length */
    float headingHalfWidthM; /* [rule] gameplay marker half-width; reads overall body width */
    bool hasCage;
    bool hasMirrors;

    /* ---- emergent form weights: 0 = absent, 1 = fully expressed ---- */
    float hoodBulgeStrength; /* [rule] 0..1; reads engineDisplacementL, massEngineXM */
    float bedLengthM;        /* [identity] open bed forward from the tail; reads
                                            spec bedLengthM, clamped to the space behind
                                            the greenhouse. 0 = no bed. */
    float pickupBedWeight;   /* [rule] 0..1 expression strength; reads bedLengthM */
    float vanWindowWeight;   /* [rule] 0..1; reads heightOverallM, cowlXM, backlightXM */
    float openWheelWeight;   /* [rule] 0..1; reads trackWidth[Front|Rear]M vs widthOverallM */
    float
        raceDetailWeight; /* [rule] 0..1; composite of low mass/size, high grip, strong aero */
    bool hasTowHook;      /* [rule] race detail marker; reads raceDetailWeight */
    bool hasHoodPins;     /* [rule] race detail marker; reads raceDetailWeight */
    float stripeWeight;   /* [rule] 0..1; reads strip01 latent */
    float heightVisual;   /* [rule] 0..1 body-height visual cue; reads heightOverallM */

    /* ---- palette; arbitrary but stable per spec, excluded from the signature ---- */
    Color body;
    Color bodyShade;
    Color cabin;
    Color glass;
    Color outline;
    Color tire;
    Color tireSidewall; /* slightly different shade for sidewall vs tread ring */
    Color rim;
    Color disc;
    Color accent;  /* bodywork appendages: wing, canards, tow hook, hood pins — in-hue */
    Color heading; /* [gameplay] L9 facing marker; deliberately the body's complement */
    Color lamp;
    Color
        stripeColor; /* [decorative] stripe colour from seed bits; geometry from stripeWeight */

    CarLatents latents;
} CarVisual;

/* The grammar. Pure and total: any spec that passes vehicle_spec_is_valid() yields finite,
 * non-negative dimensions. A NULL spec or out pointer is a no-op. Deriving twice from the same
 * spec produces a bit-identical result, which the `car-visual` scenario asserts. */
void car_visual_derive(const VehicleSpec *spec, CarVisual *out);

/* The style axes on their own, for diagnostics and documentation. */
CarLatents car_visual_latents(const VehicleSpec *spec);

/* ------------------------------------------------------------------------- signature ----
 *
 * A feature vector used by the `corpus` scenario to report WHICH feature makes two cars too
 * similar. The authoritative distinctness test is a pixel comparison of two rasters; this
 * vector exists because a pixel count cannot name the culprit and this can.
 *
 * Every component is normalised to approximate VISIBLE METRES, so the threshold has a physical
 * meaning: at the ~13.2 px/m the game draws at (PIXELS_PER_METER 24 * CAMERA_BASE_ZOOM 0.55),
 * 0.08 m is almost exactly one screen pixel. Colour contributes nothing, by design — shape has
 * to carry the difference.
 *
 * INDEPENDENCE RULE: every component must measure an independent visible difference. A
 * component that merely restates an upstream driver already represented by rendered geometry
 * (e.g. a presence flag duplicating a diameter the rasterizer gates on, or the composite weight
 * that drove it) inflates L2 distance without adding information. Such slots are retired to 0
 * — never reordered — so the remaining components each carry a genuinely independent axis.
 */
int car_visual_signature_count(void);
const char *car_visual_signature_component_name(int index);

/* The capacity every caller sizes its signature buffer to. It is a ceiling, not the count:
 * car_visual_signature_count() is authoritative and grows as components are appended, and a
 * caller that hard-codes today's count silently stops measuring the components added after it.
 * Kept generously above the current count so appending a phase's worth of components is a
 * one-line change here, not a hunt through every buffer. */
#define CAR_SIGNATURE_MAX 128

/* Writes car_visual_signature_count() floats into out. Returns the number written, or 0 if
 * out is NULL or cap is too small. */
int car_visual_signature(const CarVisual *visual, float *out, int cap);

/* The colour seed: an FNV-1a hash over a fixed, explicitly listed set of spec fields — never
 * over the raw struct bytes, which would fold in padding and make the result depend on the
 * compiler. Exposed so tests can assert that colour and only colour depends on it. */
uint32_t car_visual_colour_seed(const VehicleSpec *spec);

/* ------------------------------------------------------------------------- bake key ----
 *
 * A cache key for a baked sprite: FNV-1a over every drawn field of CarVisual, named one at a
 * time. src/render/render.c rebakes its GPU textures when this changes and does nothing when it does
 * not, which is what keeps a spec edit in the Physics Lab cheap and a still car free.
 *
 * WHY OVER CarVisual AND NOT OVER VehicleSpec. CarVisual is exactly what the rasterizer
 * reads, so a key over it cannot miss a driver: if the picture would change, this changes. A
 * key over the spec would have to re-list which fields the grammar happens to consume, and
 * would go stale the first time a rule was rewired.
 *
 * WHY NOT memcmp OR A HASH OF THE RAW BYTES. Both structs contain padding, whose contents are
 * unspecified — a key built from them could differ between two identical cars, or between two
 * compilers, and would rebake for no reason. Every field below is hashed by name, floats
 * through their exact bit pattern. Adding a field to CarVisual means adding it here; the
 * `car-visual` scenario asserts every designated visual driver moves the key, which is what
 * catches the omission.
 *
 * This key MAY invalidate a cache and MAY seed colour. It may not produce geometry — see the
 * no-hashing rule at the top of this header. */
uint32_t car_visual_bake_key(const CarVisual *visual);

#endif /* DRIFTY_CAR_VISUAL_H */
