/*
 * appearance_tests.c — the vehicle-appearance gates: purity, totality, sensitivity, scale
 * independence, and all-pairs corpus distinctness.
 *
 * The canvas, the metrics and the floors come from support/appearance_metrics.h so that these
 * scenarios and --measure-sweep cannot drift apart.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "support/test_harness.h"
#include "support/simulation_fixture.h"
#include "test_scenarios.h"
#include "scenario_shared.h"
#include "support/appearance_metrics.h"
#include "support/car_sheet.h"
#include "dev/car_corpus.h"
#include "render/car_appearance.h"
#include "render/car_visual.h"
#include "render/car_visual_raster.h"
#include "render/vehicle_effects.h"
#include "core/config.h"
#include "dev/dev_params.h"
#include "dev/dev_replay.h"
#include "dev/dev_scenario.h"
#include "dev/dev_state.h"
#include "physics/drivetrain.h"
#include "dev/failure_bundle.h"
#include "physics/surface.h"
#include "game/game.h"
#include "game/input.h"
#include "core/math_utils.h"
#include "game/particle.h"
#include "physics/physics.h"
#include "render/render.h"
#include "game/replay.h"
#include "game/telemetry.h"
#include "platform/timestep.h"
#include "physics/tire.h"
#include "core/units.h"

/* ------------------------------------------------------------------------------------- */
/* Vehicle appearance: purity, totality, sensitivity, and corpus distinctness              */
/* ------------------------------------------------------------------------------------- */

/* The designated visual drivers: the registry keys the grammar promises to be sensitive to.
 *
 * One key that might be expected here is deliberately absent: `wheel.offset`. With
 * body.track_* primary, an ET offset is already folded into the
 * track the hubs sit at, so a grammar that read it again would be double-counting a quantity
 * it has already consumed. car_visual.h states that where the field is described.
 *
 * At file scope because two assertions use the list — the sensitivity sweep and the bake-key
 * coverage check — and a driver that only one of them knew about would be a gap. */
static const char *const kVisualDrivers[] = {
    "body.wheelbase",          /* axle span, and the body length that follows it   */
    "body.front_overhang",     /* nose length                                      */
    "body.rear_overhang",      /* tail length                                      */
    "body.width_overall",      /* silhouette width and fender flare                */
    "body.height_overall",     /* roof share of the plan area, glass band, windows */
    "body.cowl_x",             /* windscreen station                               */
    "body.backlight_x",        /* rear glass station, and the deck behind it       */
    "body.bed_length",         /* open cargo bed; declared, not inferred           */
    "body.nose_width",         /* foremost hull station: the drawn nose width       */
    "body.tail_width",         /* rearmost hull station: the drawn tail width       */
    "body.shoulder_x",         /* where the body reaches its maximum width          */
    "body.fender_flare_front", /* declared front arch flare, on top of the derived  */
    "body.fender_flare_rear",  /* declared rear arch flare                          */
    "body.roof_start_x",       /* explicit forward roof station                     */
    "body.roof_end_x",         /* explicit aft roof station                         */
    "body.roof_width",         /* full physical roof width                          */
    "body.windscreen_rake",    /* top-down windscreen projection                    */
    "body.backlight_rake",     /* top-down rear-screen projection                   */
    "body.side_window_count",  /* declared pane segmentation                        */
    "body.quarter_window",     /* declared rear quarter glass                       */
    "body.sunroof_length",     /* declared roof glass                               */
    "body.door_count",         /* pillar arrangement                                */
    "body.cabin_rows",         /* rearward package from mass.driver_x               */
    "body.roof_type",          /* fixed, targa, or convertible roof panels          */

    "body.track_front",         /* front stance                                     */
    "body.track_rear",          /* rear stance                                      */
    "body.ride_height_front",   /* front arch clearance                             */
    "mass.engine_x",            /* CG, hood bulge, and the whole layout read        */
    "mass.driver_x",            /* the glasshouse has to contain the driver         */
    "tire.section_width_front", /* front tire width and diameter                    */
    "tire.section_width_rear",  /* rear tire width and diameter                     */
    "tire.aspect_front",        /* front sidewall and overall diameter              */
    "tire.aspect_rear",         /* rear sidewall and overall diameter               */
    "tire.rim_diameter_front",  /* front rim, and the tire wrapped around it        */
    "tire.rim_diameter_rear",   /* rear rim                                         */
    "aero.lift_rear",           /* tail taper and the wing on it                    */
    "engine.cylinders",         /* exhaust count                                    */
};
#define CV_VISUAL_DRIVER_COUNT ((int)(sizeof(kVisualDrivers) / sizeof(kVisualDrivers[0])))

/* Issue #9: every float field finite; dimensions non-negative; latents in [0, 1]. */
static bool cv_visual_fields_sane(const CarVisual *v)
{
    if (v == NULL) return false;

#define CV_FIN(x)                         \
    do {                                  \
        if (!isfinite((x))) return false; \
    } while (0)
#define CV_NN(x)                      \
    do {                              \
        CV_FIN(x);                    \
        if ((x) < 0.0f) return false; \
    } while (0)
#define CV_01(x)                                    \
    do {                                            \
        CV_FIN(x);                                  \
        if ((x) < 0.0f || (x) > 1.0f) return false; \
    } while (0)

    for (int s = 0; s < CAR_HULL_STATIONS; s++) {
        CV_FIN(v->hull[s].xM);
        CV_NN(v->hull[s].halfWidthM);
    }
    CV_NN(v->lengthM);
    CV_NN(v->widthM);
    CV_NN(v->wheelbaseM);
    CV_NN(v->frontOverhangM);
    CV_NN(v->rearOverhangM);

    CV_FIN(v->cabinCentreXM);
    CV_NN(v->cabinLengthM);
    CV_NN(v->cabinHalfWidthM);
    CV_FIN(v->windscreenXM);
    CV_FIN(v->backlightXM);
    CV_FIN(v->roofStartXM);
    CV_FIN(v->roofEndXM);
    CV_NN(v->roofWidthM);
    CV_NN(v->roofLengthM);
    CV_NN(v->windscreenLengthM);
    CV_NN(v->backlightLengthM);
    CV_NN(v->sideWindowBandWidthM);
    CV_NN(v->roofHighlightWidthM);
    CV_01(v->roofHighlightLengthScale);
    CV_NN(v->quarterWindow.lengthM);
    CV_NN(v->sunroof.lengthM);
    for (int i = 0; i < CAR_ROOF_PANELS_MAX; i++) {
        CV_FIN(v->roofPanels[i].centreXM);
        CV_NN(v->roofPanels[i].lengthM);
    }
    for (int i = 0; i < CAR_SIDE_WINDOWS_MAX; i++) {
        CV_FIN(v->sideWindows[i].centreXM);
        CV_NN(v->sideWindows[i].lengthM);
    }

    for (int i = 0; i < WHEEL_COUNT; i++) {
        CV_FIN(v->wheels[i].centreM.x);
        CV_FIN(v->wheels[i].centreM.y);
        CV_NN(v->wheels[i].diameterM);
        CV_NN(v->wheels[i].widthM);
        CV_NN(v->wheels[i].rimDiameterM);
        CV_NN(v->wheels[i].discDiameterM);
        CV_FIN(v->wheels[i].staticAngleRad);
    }
    CV_NN(v->archFlareM);

    CV_NN(v->wingSpanM);
    CV_NN(v->wingChordM);
    CV_FIN(v->wingXM);
    CV_NN(v->splitterProtrusionM);
    CV_NN(v->splitterWidthM);
    CV_NN(v->mirrorOffsetM);
    CV_NN(v->exhaustBoreM);
    if (v->exhaustCount < 0) return false;
    CV_NN(v->towHookDiameterM);
    CV_FIN(v->towHookXM);
    CV_NN(v->hoodPinDiameterM);
    CV_NN(v->headingLengthM);
    CV_NN(v->headingHalfWidthM);

    CV_01(v->latents.mass01);
    CV_01(v->latents.size01);
    CV_01(v->latents.low01);
    CV_01(v->latents.grip01);
    CV_01(v->latents.balance01);
    CV_01(v->latents.power01);
    CV_01(v->latents.aero01);
    CV_01(v->latents.sport01);
    CV_01(v->latents.strip01);

#undef CV_FIN
#undef CV_NN
#undef CV_01
    return v->lengthM > 0.0f && v->widthM > 0.0f;
}

static void scenario_car_visual(void)
{
    VehicleSpec spec;
    CarVisual a, b;

    /* --- purity: same input, byte-identical CarVisual / signature / raster --- */
    vehicle_spec_set_default(&spec);
    car_visual_derive(&spec, &a);
    car_visual_derive(&spec, &b);
    check(memcmp(&a, &b, sizeof(CarVisual)) == 0,
          "car_visual_derive is pure: identical specs give identical visuals");

    {
        float sa[CAR_SIGNATURE_MAX], sb[CAR_SIGNATURE_MAX];
        const int n = car_visual_signature_count();
        check(n > 0 && n <= (int)(sizeof(sa) / sizeof(sa[0])),
              "signature component count is in range");
        check(car_visual_signature(&a, sa, n) == n && car_visual_signature(&b, sb, n) == n,
              "signature writes every component twice");
        check(memcmp(sa, sb, (size_t)n * sizeof(float)) == 0,
              "car_visual_signature is pure: identical visuals give identical signatures");
    }

    {
        const CarRasterInfo info = car_raster_info(&a, CV_TEST_PX_PER_M, 2);
        const size_t bytes = car_raster_bytes(info);
        const size_t labelBytes = (size_t)info.width * (size_t)info.height;
        unsigned char *ra = (unsigned char *)malloc(bytes);
        unsigned char *rb = (unsigned char *)malloc(bytes);
        unsigned char *la = (unsigned char *)malloc(labelBytes);
        unsigned char *lb = (unsigned char *)malloc(labelBytes);
        check(ra != NULL && rb != NULL && la != NULL && lb != NULL && bytes > 0,
              "purity raster buffers allocated");
        if (ra != NULL && rb != NULL && la != NULL && lb != NULL) {
            check(car_raster_draw(&a, info, ra, bytes) && car_raster_draw(&a, info, rb, bytes),
                  "repeated RGBA rasters succeed");
            check(memcmp(ra, rb, bytes) == 0,
                  "car_raster_draw is pure: identical visuals give bit-identical RGBA");
            check(car_raster_draw_labels(&a, info, la, labelBytes) &&
                      car_raster_draw_labels(&a, info, lb, labelBytes),
                  "repeated label rasters succeed");
            check(
                memcmp(la, lb, labelBytes) == 0,
                "car_raster_draw_labels is pure: identical visuals give bit-identical labels");
        }
        free(ra);
        free(rb);
        free(la);
        free(lb);
    }

    /* --- the wheel centres ARE the simulation's wheel positions, not a lookalike --- */
    {
        VehicleState state;
        VehicleDerived derived;
        VehicleRenderState renderState;
        vehicle_state_reset(&spec, &state, &derived, &renderState);
        bool matched = true;
        for (int i = 0; i < WHEEL_COUNT; i++) {
            if (fabsf(a.wheels[i].centreM.x - state.wheels[i].localPositionM.x) > 1e-6f ||
                fabsf(a.wheels[i].centreM.y - state.wheels[i].localPositionM.y) > 1e-6f) {
                matched = false;
            }
        }
        check(matched, "drawn wheel centres equal vehicle.c set_wheel_positions()");
        check(fabsf(a.wheelbaseM - (spec.cgToFrontM + spec.cgToRearM)) < 1e-6f,
              "drawn wheelbase equals the simulated wheelbase");
        check(fabsf(a.wheels[WHEEL_FRONT_LEFT].diameterM - 2.0f * spec.wheelRadiusFrontM) <
                  1e-6f,
              "drawn front tire diameter equals 2 * wheelRadiusFrontM");
        check(fabsf(a.wheels[WHEEL_REAR_LEFT].diameterM - 2.0f * spec.wheelRadiusRearM) < 1e-6f,
              "drawn rear tire diameter equals 2 * wheelRadiusRearM");
        check(fabsf(a.widthM - 2.0f * spec.bodyHalfWidthM) < 1e-6f,
              "drawn body width equals the collision half-width doubled");
    }

    /* --- totality: every declared range corner yields finite, sane geometry --- */
    {
        int bad = 0;
        for (int p = 0; p < dev_params_count(); p++) {
            const DevParameter *param = dev_param_at(p);
            for (int corner = 0; corner < 2; corner++) {
                VehicleSpec probe;
                vehicle_spec_set_default(&probe);
                dev_param_set(&probe, param, corner == 0 ? param->minimum : param->maximum);

                CarVisual v;
                car_visual_derive(&probe, &v);
                if (!cv_visual_fields_sane(&v)) {
                    if (bad == 0) {
                        printf("      first bad corner: %s = %g\n", param->name,
                               (double)(corner == 0 ? param->minimum : param->maximum));
                    }
                    bad++;
                }
            }
        }
        check(bad == 0,
              "every registry range corner produces finite, bounded CarVisual fields");
    }

    /* --- monotonicity: the obvious knobs move the obvious way. These are what stop a
     * distinctness failure being "fixed" by injecting noise into the grammar. --- */
    {
        VehicleSpec lo, hi;
        CarVisual vlo, vhi;

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.tireRimDiameterFrontIn = 14.0f;
        lo.tireRimDiameterRearIn = 14.0f;
        hi.tireRimDiameterFrontIn = 20.0f;
        hi.tireRimDiameterRearIn = 20.0f;
        vehicle_spec_refresh_derived(&lo);
        vehicle_spec_refresh_derived(&hi);
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.wheels[0].diameterM > vlo.wheels[0].diameterM,
              "a larger rim designation draws a larger tire");

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.widthOverallM = 1.40f;
        hi.widthOverallM = 2.20f;
        vehicle_spec_refresh_derived(&lo);
        vehicle_spec_refresh_derived(&hi);
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.widthM > vlo.widthM, "a wider body draws a wider silhouette");

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.wheelbaseM = 2.10f;
        hi.wheelbaseM = 3.10f;
        vehicle_spec_refresh_derived(&lo);
        vehicle_spec_refresh_derived(&hi);
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.wheelbaseM > vlo.wheelbaseM, "a longer wheelbase draws a longer axle span");
        check(vhi.lengthM > vlo.lengthM, "a longer wheelbase draws a longer body");

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.frontOverhangM = 0.40f;
        hi.frontOverhangM = 1.40f;
        vehicle_spec_refresh_derived(&lo);
        vehicle_spec_refresh_derived(&hi);
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.frontOverhangM > vlo.frontOverhangM,
              "a longer front overhang draws a longer nose");
        check(vhi.lengthM > vlo.lengthM, "a longer front overhang draws a longer body");

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.maxBrakeTorqueNm = 500.0f;
        hi.maxBrakeTorqueNm = 6000.0f;
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.wheels[0].discDiameterM > vlo.wheels[0].discDiameterM,
              "more brake torque draws a larger disc");

        /* Moving the engine rearward moves the CG rearward, and everything drawn is measured
         * from the CG. So the front axle ends up further AHEAD of the origin, and the cabin
         * with it — the body-frame shift is the honest consequence, not a bug. What must not
         * happen is the hood bulge growing on a car whose engine is now behind the driver. */
        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.massEngineXM = 1.60f;  /* nose-forward */
        hi.massEngineXM = -1.20f; /* behind the driver */
        vehicle_spec_refresh_derived(&lo);
        vehicle_spec_refresh_derived(&hi);
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(hi.cgToFrontM > lo.cgToFrontM && vhi.wheels[WHEEL_FRONT_LEFT].centreM.x >
                                                   vlo.wheels[WHEEL_FRONT_LEFT].centreM.x,
              "a rearward engine puts the CG back, so the front axle is drawn further ahead");
        check(vhi.hoodBulgeStrength < vlo.hoodBulgeStrength,
              "a rear-mounted engine does not bulge the hood the way a front one does");

        /* More rear downforce demand must not shrink the wing. */
        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.aeroLiftCoefRear = -0.60f;
        hi.aeroLiftCoefRear = -2.60f;
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.wingSpanM >= vlo.wingSpanM && vhi.wingChordM >= vlo.wingChordM,
              "more rear downforce demand does not reduce the wing");
        check(vhi.wingSpanM > vlo.wingSpanM, "and a large increase actually grows it");

        /* Terminating the greenhouse earlier must not shorten the deck behind it. */
        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.backlightXM = -0.20f; /* glass runs well back */
        hi.backlightXM = -1.40f; /* glass stops early, long open rear */
        vehicle_spec_refresh_derived(&lo);
        vehicle_spec_refresh_derived(&hi);
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.backlightXM < vlo.backlightXM,
              "an earlier backlight station draws the rear glass further forward");
        check(vhi.pickupBedWeight <= vlo.pickupBedWeight + 1e-6f,
              "and does not increase the share of the body read as a bed");

        /* More ride height must not reduce the arch clearance above the tire. */
        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.rideHeightFrontM = 0.06f;
        hi.rideHeightFrontM = 0.32f;
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.wheels[WHEEL_FRONT_LEFT].archGapM > vlo.wheels[WHEEL_FRONT_LEFT].archGapM,
              "a higher ride height draws more wheel-arch clearance");

        /* A taller body must not shrink the roof it is supposed to be roofing. */
        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.heightOverallM = 1.10f;
        hi.heightOverallM = 2.90f;
        vehicle_spec_refresh_derived(&lo);
        vehicle_spec_refresh_derived(&hi);
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.cabinHalfWidthM > vlo.cabinHalfWidthM,
              "a taller body roofs over more of its own plan area");

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.windscreenRakeRad = VEH_WINDSCREEN_RAKE_MIN_RAD;
        hi.windscreenRakeRad = VEH_WINDSCREEN_RAKE_MAX_RAD;
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.windscreenLengthM > vlo.windscreenLengthM,
              "more windscreen rake exposes more glass from directly above");

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.cabinRows = 1.0f;
        hi.cabinRows = 3.0f;
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.backlightXM < vlo.backlightXM && vhi.cabinLengthM > vlo.cabinLengthM,
              "additional cabin rows extend the package rearward from mass.driver_x");

        vehicle_spec_set_default(&lo);
        lo.roofWidthM = 1.13f;
        car_visual_derive(&lo, &vlo);
        check(fabsf(vlo.roofWidthM - lo.roofWidthM) < 1e-6f,
              "drawn roof width equals the declared full physical width");

        vehicle_spec_set_default(&lo);
        lo.roofType = (float)VEH_ROOF_FIXED;
        car_visual_derive(&lo, &vlo);
        lo.roofType = (float)VEH_ROOF_TARGA;
        car_visual_derive(&lo, &vhi);
        check(vlo.roofPanelCount == 1 && vhi.roofPanelCount == 2,
              "roof type derives fixed and targa panel arrangements");

        vehicle_spec_set_default(&lo);
        lo.windscreenRakeRad = NAN;
        lo.backlightRakeRad = NAN;
        car_visual_derive(&lo, &vlo);
        check(isfinite(vlo.windscreenLengthM) && isfinite(vlo.backlightLengthM),
              "invalid live-edited rake values fall back to finite default projections");
    }

    /* --- sensitivity ------------------------------------------------------------------
     *
     * The designated visual drivers, each perturbed across its whole declared registry range
     * from the canonical stock spec. Both metrics must move: the pixel metric proves the
     * change reaches the picture, the L-infinity metric proves it reaches a NAMED feature and
     * by at least one screen pixel. A rule that was deleted, mis-wired, or shadowed by a
     * fallback branch scores 0.0000 here and cannot hide.
     *
     * The list is kVisualDrivers at file scope; see the comment there for what is on it and
     * what is deliberately not. --- */
    {
        const int driverCount = CV_VISUAL_DRIVER_COUNT;
        check(driverCount >= 20, "at least twenty designated visual drivers (%d)", driverCount);

        const CarRasterInfo canvas = test_car_shared_canvas(CV_TEST_PX_PER_M);
        const size_t pixels = (size_t)canvas.width * (size_t)canvas.height;
        unsigned char *la = (unsigned char *)malloc(pixels);
        unsigned char *lb = (unsigned char *)malloc(pixels);
        check(la != NULL && lb != NULL && pixels > 0, "sensitivity canvas allocated");

        if (la != NULL && lb != NULL) {
            int dead = 0, missing = 0;
            for (int d = 0; d < driverCount; d++) {
                const DevParameter *param = dev_param_find(kVisualDrivers[d]);
                if (param == NULL) {
                    printf("      designated visual driver not in the registry: %s\n",
                           kVisualDrivers[d]);
                    missing++;
                    continue;
                }

                VehicleSpec lo, hi;
                vehicle_spec_set_default(&lo);
                vehicle_spec_set_default(&hi);
                dev_param_set(&lo, param, param->minimum);
                dev_param_set(&hi, param, param->maximum);

                CarVisual vlo, vhi;
                car_visual_derive(&lo, &vlo);
                car_visual_derive(&hi, &vhi);

                test_car_labels_for_spec(&lo, canvas, la, pixels);
                test_car_labels_for_spec(&hi, canvas, lb, pixels);
                const float diff = car_raster_difference(la, lb, canvas.width, canvas.height);

                int worstComponent = 0;
                const float linf = test_car_signature_linf(&vlo, &vhi, &worstComponent);

                if (diff < CV_MIN_SENSITIVITY_DIFF || linf < CV_MIN_LINF) {
                    printf("      dead visual axis: %s over [%g, %g] moves %.4f of pixels"
                           " and %.4f m in '%s'\n",
                           param->name, (double)param->minimum, (double)param->maximum,
                           (double)diff, (double)linf,
                           car_visual_signature_component_name(worstComponent));
                    dead++;
                }
            }
            check(missing == 0, "every designated visual driver exists in the registry");
            check(dead == 0, "every designated visual driver moves both the pixels and a"
                             " named feature");
        }
        free(la);
        free(lb);
    }

    /* --- composable parts: the game draws the body and each wheel as separate sprites so
     * the front wheels can steer, and it must be the SAME picture. Body and whole-car
     * rasters are compared pixel for pixel; every disagreement has to fall on a pixel the
     * label map calls tire, sidewall, rim or disc. If the body sprite ever grew its own
     * geometry — a second, production-only grammar — this is what would catch it. --- */
    {
        int wrongPixels = 0, emptyWheels = 0, tested = 0;
        const int step = (car_corpus_count() > 12) ? (car_corpus_count() / 12) : 1;

        for (int c = 0; c < car_corpus_count(); c += step) {
            VehicleSpec probe;
            CarVisual v;
            if (!car_corpus_spec(c, &probe)) continue;
            car_visual_derive(&probe, &v);

            const CarRasterInfo info = car_raster_info(&v, CV_TEST_PX_PER_M, 1);
            const CarRasterInfo bodyInfo =
                car_raster_part_info(&v, CAR_RASTER_PART_BODY, 0, CV_TEST_PX_PER_M, 1);
            if (info.width != bodyInfo.width || info.height != bodyInfo.height ||
                info.originXPx != bodyInfo.originXPx || info.originYPx != bodyInfo.originYPx) {
                wrongPixels++;
                continue;
            }

            const size_t bytes = car_raster_bytes(info);
            const size_t labelBytes = (size_t)info.width * (size_t)info.height;
            unsigned char *all = (unsigned char *)malloc(bytes);
            unsigned char *body = (unsigned char *)malloc(bytes);
            unsigned char *labels = (unsigned char *)malloc(labelBytes);
            if (all == NULL || body == NULL || labels == NULL) {
                free(all);
                free(body);
                free(labels);
                continue;
            }

            if (car_raster_draw_part(&v, CAR_RASTER_PART_ALL, 0, info, all, bytes) &&
                car_raster_draw_part(&v, CAR_RASTER_PART_BODY, 0, info, body, bytes) &&
                car_raster_draw_labels(&v, info, labels, labelBytes)) {
                int differing = 0;
                for (size_t p = 0; p < labelBytes; p++) {
                    if (memcmp(all + p * CAR_RASTER_BPP, body + p * CAR_RASTER_BPP,
                               CAR_RASTER_BPP) == 0) {
                        continue;
                    }
                    differing++;
                    const unsigned char lab = labels[p];
                    if (lab != CAR_LABEL_TIRE && lab != CAR_LABEL_TIRE_SIDEWALL &&
                        lab != CAR_LABEL_RIM && lab != CAR_LABEL_DISC) {
                        if (wrongPixels == 0) {
                            char id[128];
                            car_corpus_id(c, id, sizeof(id));
                            printf("      body sprite differs from the whole car outside the"
                                   " wheels: %s, label %d\n",
                                   id, (int)lab);
                        }
                        wrongPixels++;
                    }
                }
                if (differing == 0) emptyWheels++;
                tested++;
            }
            free(all);
            free(body);
            free(labels);
        }

        check(tested > 0, "composability sampled %d corpus vehicles", tested);
        check(wrongPixels == 0,
              "the body sprite is the whole car minus its tires, pixel for pixel");
        check(emptyWheels == 0, "and the tires are actually drawn in the whole-car raster");
    }

    /* --- the wheel sprite pivots on its hub and does not bake in its static angle --- */
    {
        CarVisual v;
        vehicle_spec_set_default(&spec);
        car_visual_derive(&spec, &v);

        const CarRasterInfo w0 = car_raster_part_info(&v, CAR_RASTER_PART_WHEEL,
                                                      WHEEL_FRONT_LEFT, CV_TEST_PX_PER_M, 1);
        check(w0.width > 0 && w0.height > 0, "a wheel sprite has a size");
        check(fabsf(w0.originXPx - 0.5f * (float)w0.width) < 1e-6f &&
                  fabsf(w0.originYPx - 0.5f * (float)w0.height) < 1e-6f,
              "the wheel sprite pivots on its own hub, at the centre of its buffer");

        const size_t bytes = car_raster_bytes(w0);
        unsigned char *buf = (unsigned char *)malloc(bytes);
        if (buf != NULL) {
            check(car_raster_draw_part(&v, CAR_RASTER_PART_WHEEL, WHEEL_FRONT_LEFT, w0, buf,
                                       bytes),
                  "a wheel sprite rasterizes");
            int opaque = 0;
            for (size_t p = 3; p < bytes; p += CAR_RASTER_BPP) {
                if (buf[p] > 0) opaque++;
            }
            check(opaque > 0, "and it is not empty (%d covered pixels)", opaque);
            check(!car_raster_draw_part(&v, CAR_RASTER_PART_WHEEL, WHEEL_COUNT, w0, buf, bytes),
                  "an out-of-range wheel index is refused rather than read out of bounds");
            free(buf);
        }
    }

    /* --- the bake key: what src/render/render.c uses to decide whether to rebuild its textures ---
     *
     * Two properties, and the whole texture cache rests on them: an unchanged spec must not
     * rebake (or a still car pays for a bake every frame), and any change a player could see
     * MUST rebake (or the car goes stale after a Physics Lab edit). --- */
    {
        VehicleSpec a1, a2;
        CarVisual v1, v2;
        vehicle_spec_set_default(&a1);
        vehicle_spec_set_default(&a2);
        car_visual_derive(&a1, &v1);
        car_visual_derive(&a2, &v2);
        check(car_visual_bake_key(&v1) == car_visual_bake_key(&v2),
              "the bake key is pure: an unchanged spec never triggers a rebake");
        check(car_visual_bake_key(NULL) == 0u,
              "a NULL visual yields no key rather than a read");

        int deadKeys = 0;
        for (int d = 0; d < CV_VISUAL_DRIVER_COUNT; d++) {
            const DevParameter *param = dev_param_find(kVisualDrivers[d]);
            if (param == NULL) continue;

            VehicleSpec lo, hi;
            CarVisual vlo, vhi;
            vehicle_spec_set_default(&lo);
            vehicle_spec_set_default(&hi);
            dev_param_set(&lo, param, param->minimum);
            dev_param_set(&hi, param, param->maximum);
            car_visual_derive(&lo, &vlo);
            car_visual_derive(&hi, &vhi);

            if (car_visual_bake_key(&vlo) == car_visual_bake_key(&vhi)) {
                if (deadKeys == 0) {
                    printf("      bake key ignores a visual driver: %s\n", param->name);
                }
                deadKeys++;
            }
        }
        check(deadKeys == 0, "every designated visual driver changes the bake key");

        /* Distinct cars must not collide onto one cached sprite. */
        int collisions = 0;
        const int count = car_corpus_count();
        for (int i = 0; i < count && collisions == 0; i++) {
            VehicleSpec si;
            CarVisual vi;
            if (!car_corpus_spec(i, &si)) continue;
            car_visual_derive(&si, &vi);
            const uint32_t ki = car_visual_bake_key(&vi);
            for (int j = i + 1; j < count; j++) {
                VehicleSpec sj;
                CarVisual vj;
                if (!car_corpus_spec(j, &sj)) continue;
                car_visual_derive(&sj, &vj);
                if (car_visual_bake_key(&vj) == ki) {
                    char ida[128], idb[128];
                    car_corpus_id(i, ida, sizeof(ida));
                    car_corpus_id(j, idb, sizeof(idb));
                    printf("      bake key collision: '%s' and '%s'\n", ida, idb);
                    collisions++;
                    break;
                }
            }
        }
        check(collisions == 0, "no two corpus vehicles share a bake key");
    }

    /* --- scale independence: the grammar is metres, the raster is pixels, and only the
     * second one may notice a change of scale. Mirrors scenario_renderscale. --- */
    {
        CarVisual v16, v32;
        vehicle_spec_set_default(&spec);
        car_visual_derive(&spec, &v16);

        const CarRasterInfo one = car_raster_info(&v16, 16.0f, 0);
        const CarRasterInfo two = car_raster_info(&v16, 32.0f, 0);
        check(two.width >= one.width * 2 - 3 && two.width <= one.width * 2 + 3,
              "doubling the raster scale doubles the sprite width");
        check(two.height >= one.height * 2 - 3 && two.height <= one.height * 2 + 3,
              "doubling the raster scale doubles the sprite height");

        /* car_visual_derive takes no scale at all, so this cannot regress by accident — but
         * it is exactly the property a future "just scale it in the grammar" shortcut would
         * break, and it costs one derive to pin down. */
        car_visual_derive(&spec, &v32);
        check(memcmp(&v16, &v32, sizeof(CarVisual)) == 0,
              "CarVisual is identical regardless of the scale anything is rasterized at");

        float s16[CAR_SIGNATURE_MAX], s32[CAR_SIGNATURE_MAX];
        const int n = car_visual_signature_count();
        if (car_visual_signature(&v16, s16, n) == n &&
            car_visual_signature(&v32, s32, n) == n) {
            check(memcmp(s16, s32, (size_t)n * sizeof(float)) == 0,
                  "the diagnostic signature is independent of raster scale");
        }
        check(fabsf(v16.lengthM - (spec.cgToFrontM + spec.frontOverhangM + spec.cgToRearM +
                                   spec.rearOverhangM)) < 1e-6f,
              "drawn length stays a physical quantity, in metres");
    }
}

static void scenario_corpus(void)
{
    const int count = car_corpus_count();
    check(count >= 50, "the corpus holds at least 50 vehicles (have %d)", count);
    check(count >= 90 && count <= 110,
          "the completed corpus is approximately 100 vehicles (have %d)", count);

    /* --- the corpus is a pure function of the index: asking twice gives the same fleet ---
     * The sampled group is generated lazily by rejection sampling, which is exactly the kind
     * of code that grows an order dependency by accident. */
    {
        int unstable = 0;
        for (int i = 0; i < count; i++) {
            VehicleSpec first, again;
            if (!car_corpus_spec(i, &first)) {
                unstable++;
                continue;
            }
            /* Ask for a later entry in between, so a cache that keyed on "last index" would
             * be caught rather than flattered. */
            VehicleSpec scratch;
            (void)car_corpus_spec((i + 37) % count, &scratch);
            if (!car_corpus_spec(i, &again)) {
                unstable++;
                continue;
            }
            if (memcmp(&first, &again, sizeof(VehicleSpec)) != 0) {
                if (unstable == 0) {
                    char id[128];
                    car_corpus_id(i, id, sizeof(id));
                    printf("      corpus entry is not deterministic: %s\n", id);
                }
                unstable++;
            }
        }
        check(unstable == 0, "every corpus entry is deterministic and order-independent");
    }

    /* --- every entry is a valid, renderable car --- */
    {
        int invalid = 0;
        for (int i = 0; i < count; i++) {
            VehicleSpec spec;
            char id[128];
            car_corpus_id(i, id, sizeof(id));
            if (!car_corpus_spec(i, &spec)) {
                invalid++;
                continue;
            }
            if (!vehicle_spec_is_valid(&spec)) {
                if (invalid == 0) printf("      first invalid corpus spec: %s\n", id);
                invalid++;
            }
        }
        check(invalid == 0, "every corpus vehicle passes vehicle_spec_is_valid()");
    }

    /* --- sweep steps must not reproduce stock or collapse onto a neighbour ---
     * Regression for the midpoint-default collision: evenly spaced [min, max] steps of
     * body.cg_to_rear landed on VEH_CG_TO_REAR_M at step 2, so sweep_body_cg_to_rear_2 was
     * bit-identical to archetype_00_stock_baseline. The grammar already reads cgToRearM;
     * the bug was corpus sampling, not a disconnected mapping. */
    {
        VehicleSpec stock;
        vehicle_spec_set_default(&stock);

        int collapsed = 0;
        for (int i = 0; i < count; i++) {
            if (car_corpus_group(i) != CAR_CORPUS_SWEEP) continue;

            const char *key = car_corpus_sweep_key(i);
            const DevParameter *param = (key != NULL) ? dev_param_find(key) : NULL;
            VehicleSpec spec;
            if (param == NULL || !car_corpus_spec(i, &spec)) {
                collapsed++;
                continue;
            }

            const float value = dev_param_get(&spec, param);
            const float stockValue = dev_param_get(&stock, param);
            /* Corpus sampling excludes a >= 0.08 m window around the stock default for
             * metre-valued drivers; require at least that gap here so a midpoint collision
             * fails loudly. Neighbour spacing is allowed to be tighter — the pairwise pixel
             * test owns visual separation between adjacent steps. */
            const float minStockGap = 0.08f;
            const float minNeighbourGap = (param->step > 0.0f) ? param->step : 1e-4f;

            if (fabsf(value - stockValue) < minStockGap) {
                if (collapsed == 0) {
                    char id[128];
                    car_corpus_id(i, id, sizeof(id));
                    printf("      sweep reproduces stock: %s (%s = %g, stock %g)\n", id,
                           param->name, (double)value, (double)stockValue);
                }
                collapsed++;
                continue;
            }

            /* Neighbour on the same axis (previous step), if any. */
            if (i > 0 && car_corpus_group(i - 1) == CAR_CORPUS_SWEEP &&
                car_corpus_sweep_key(i - 1) != NULL &&
                strcmp(car_corpus_sweep_key(i - 1), key) == 0) {
                VehicleSpec prev;
                if (!car_corpus_spec(i - 1, &prev)) {
                    collapsed++;
                    continue;
                }
                const float prevValue = dev_param_get(&prev, param);
                if (fabsf(value - prevValue) < minNeighbourGap) {
                    if (collapsed == 0) {
                        char id[128], pid[128];
                        car_corpus_id(i, id, sizeof(id));
                        car_corpus_id(i - 1, pid, sizeof(pid));
                        printf("      sweep neighbours collide: %s vs %s (%s = %g / %g)\n", pid,
                               id, param->name, (double)prevValue, (double)value);
                    }
                    collapsed++;
                }
            }
        }
        check(collapsed == 0,
              "every sweep step differs from stock and from neighbouring steps on its axis");
    }

    /* --- a sweep row varies EXACTLY ONE primary key ---
     * This is the whole claim the sweep artifact makes. Comparing consecutive steps of the
     * same axis is the direct test of it: whatever base a row is drawn on, only the axis key
     * may move between its cars. */
    {
        int multiKey = 0;
        for (int i = 1; i < count; i++) {
            if (car_corpus_group(i) != CAR_CORPUS_SWEEP) continue;
            if (car_corpus_group(i - 1) != CAR_CORPUS_SWEEP) continue;
            const char *key = car_corpus_sweep_key(i);
            const char *prevKey = car_corpus_sweep_key(i - 1);
            if (key == NULL || prevKey == NULL || strcmp(key, prevKey) != 0) continue;

            VehicleSpec cur, prev;
            if (!car_corpus_spec(i, &cur) || !car_corpus_spec(i - 1, &prev)) {
                multiKey++;
                continue;
            }
            const char *firstDiff = NULL;
            const int diffs = test_car_primary_diff_count(&prev, &cur, &firstDiff);
            if (diffs != 1 || firstDiff == NULL || strcmp(firstDiff, key) != 0) {
                if (multiKey == 0) {
                    char id[128];
                    car_corpus_id(i, id, sizeof(id));
                    printf("      sweep step varies %d primary key(s), first '%s',"
                           " expected only '%s' (%s)\n",
                           diffs, (firstDiff != NULL) ? firstDiff : "-", key, id);
                }
                multiKey++;
            }
        }
        check(multiKey == 0, "each sweep row varies exactly one primary registry key");
    }

    /* --- the fleet actually spans the range the widened registry paid for ---
     * A corpus of a hundred mid-size saloons would pass every distinctness assertion and
     * still fail the point of the exercise. These bounds are the reference sheet's extremes:
     * a kei car at one end, a bus or box truck at the other. */
    {
        float minMass = 1e9f, maxMass = -1e9f;
        float minWb = 1e9f, maxWb = -1e9f;
        float minWidth = 1e9f, maxWidth = -1e9f;
        float minHeight = 1e9f, maxHeight = -1e9f;

        for (int i = 0; i < count; i++) {
            VehicleSpec spec;
            if (!car_corpus_spec(i, &spec)) continue;
            if (spec.massKg < minMass) minMass = spec.massKg;
            if (spec.massKg > maxMass) maxMass = spec.massKg;
            if (spec.wheelbaseM < minWb) minWb = spec.wheelbaseM;
            if (spec.wheelbaseM > maxWb) maxWb = spec.wheelbaseM;
            if (spec.widthOverallM < minWidth) minWidth = spec.widthOverallM;
            if (spec.widthOverallM > maxWidth) maxWidth = spec.widthOverallM;
            if (spec.heightOverallM < minHeight) minHeight = spec.heightOverallM;
            if (spec.heightOverallM > maxHeight) maxHeight = spec.heightOverallM;
        }

        check(minMass <= 800.0f && maxMass >= 6000.0f,
              "corpus mass spans kei to heavy commercial (%.0f .. %.0f kg)", (double)minMass,
              (double)maxMass);
        check(minWb <= 2.30f && maxWb >= 5.00f,
              "corpus wheelbase spans kei to bus (%.2f .. %.2f m)", (double)minWb,
              (double)maxWb);
        check(minWidth <= 1.40f && maxWidth >= 2.20f,
              "corpus width spans narrow to commercial (%.2f .. %.2f m)", (double)minWidth,
              (double)maxWidth);
        check(minHeight <= 1.20f && maxHeight >= 2.60f,
              "corpus height spans supercar to box truck (%.2f .. %.2f m)", (double)minHeight,
              (double)maxHeight);
    }

    /* --- the checked-in profiles are still the fleet the code generates ---
     * data/vehicles/corpus/ is an export for humans to read and diff, not a second source of
     * truth.
     * That only stays true if it is asserted: without this the files rot silently and the
     * next reader trusts a stale one. Regenerate with
     *     build/tests/drifty_tests.exe --generate-corpus data/vehicles/corpus                 */
    {
        int missing = 0, mismatched = 0;
        const char *firstMissing = NULL;
        static char firstMissingBuf[512];

        for (int i = 0; i < count; i++) {
            VehicleSpec expected, loaded;
            char id[128], path[512];
            if (!car_corpus_spec(i, &expected)) continue;
            car_corpus_id(i, id, sizeof(id));
            snprintf(path, sizeof(path), "data/vehicles/corpus/%s/%s.txt",
                     car_corpus_group_name(car_corpus_group(i)), id);

            vehicle_spec_set_default(&loaded);
            if (!dev_params_load(&loaded, path, NULL, NULL, NULL)) {
                if (missing == 0) {
                    snprintf(firstMissingBuf, sizeof(firstMissingBuf), "%s", path);
                    firstMissing = firstMissingBuf;
                }
                missing++;
                continue;
            }

            const char *firstDiff = NULL;
            if (test_car_primary_diff_count(&expected, &loaded, &firstDiff) != 0) {
                if (mismatched == 0) {
                    printf("      exported profile does not round-trip: %s (first '%s')\n",
                           path, (firstDiff != NULL) ? firstDiff : "?");
                }
                mismatched++;
            }
        }

        if (missing > 0) {
            printf(
                "      %d corpus profile(s) missing, first '%s' —"
                " run: build/tests/drifty_tests.exe --generate-corpus data/vehicles/corpus\n",
                missing, (firstMissing != NULL) ? firstMissing : "?");
        }
        check(missing == 0, "every corpus vehicle has a checked-in tuning profile");
        check(mismatched == 0, "every checked-in corpus profile round-trips to its spec");
    }

    /* --- all-pairs distinctness on the colour-blind label maps --- */
    {
        const CarRasterInfo canvas = test_car_shared_canvas(CV_TEST_PX_PER_M);
        const size_t pixels = (size_t)canvas.width * (size_t)canvas.height;
        unsigned char *maps = (unsigned char *)malloc(pixels * (size_t)count);
        CarVisual *visuals = (CarVisual *)malloc(sizeof(CarVisual) * (size_t)count);
        check(maps != NULL && visuals != NULL && pixels > 0, "distinctness buffers allocated");

        if (maps != NULL && visuals != NULL) {
            bool built = true;
            for (int i = 0; i < count; i++) {
                VehicleSpec spec;
                if (!car_corpus_spec(i, &spec)) {
                    built = false;
                    break;
                }
                car_visual_derive(&spec, &visuals[i]);
                if (!car_raster_draw_labels(&visuals[i], canvas, maps + pixels * (size_t)i,
                                            pixels)) {
                    built = false;
                    break;
                }
            }
            check(built, "every corpus vehicle rasterizes onto the shared canvas");

            if (built) {
                float worstDiff = 1.0f;
                int worstA = -1, worstB = -1;
                for (int i = 0; i < count; i++) {
                    for (int j = i + 1; j < count; j++) {
                        const float d = car_raster_difference(maps + pixels * (size_t)i,
                                                              maps + pixels * (size_t)j,
                                                              canvas.width, canvas.height);
                        if (d < worstDiff) {
                            worstDiff = d;
                            worstA = i;
                            worstB = j;
                        }
                    }
                }

                if (worstDiff < CV_MIN_PIXEL_DIFF && worstA >= 0) {
                    char ida[128], idb[128];
                    int worstComponent = 0;
                    car_corpus_id(worstA, ida, sizeof(ida));
                    car_corpus_id(worstB, idb, sizeof(idb));
                    const float linf = test_car_signature_linf(
                        &visuals[worstA], &visuals[worstB], &worstComponent);
                    printf("      closest pair '%s' vs '%s': %.4f of pixels differ,"
                           " largest feature gap %.4f m in '%s'\n",
                           ida, idb, (double)worstDiff, (double)linf,
                           car_visual_signature_component_name(worstComponent));
                    /* A pass/fail line cannot say WHY two cars look alike. The bundle can:
                     * both rasters, the diff, both specs as loadable profiles, and the
                     * signature components that came closest. */
                    if (test_harness_bundles_enabled()) {
                        (void)car_sheet_write_pair_failure(CV_FAILURE_DIR, worstA, worstB,
                                                           CV_TEST_PX_PER_M);
                    }
                }
                check(worstDiff >= CV_MIN_PIXEL_DIFF,
                      "every corpus pair differs in >= %.1f%% of pixels (closest %.2f%%)",
                      (double)(CV_MIN_PIXEL_DIFF * 100.0f), (double)(worstDiff * 100.0f));

                /* The diagnostic vector must agree that the closest pair is separable, so a
                 * failure can always name a feature rather than only a pixel count. L-infinity
                 * asks for one clearly different feature; L2 asks that the cars are not merely
                 * a rounding error apart across the whole vector. */
                float worstLinf = 1e9f, worstL2 = 1e9f;
                int l2A = -1, l2B = -1;
                for (int i = 0; i < count; i++) {
                    for (int j = i + 1; j < count; j++) {
                        const float linf =
                            test_car_signature_linf(&visuals[i], &visuals[j], NULL);
                        if (linf < worstLinf) worstLinf = linf;
                        const float l2 = test_car_signature_l2(&visuals[i], &visuals[j]);
                        if (l2 < worstL2) {
                            worstL2 = l2;
                            l2A = i;
                            l2B = j;
                        }
                    }
                }
                check(worstLinf >= CV_MIN_LINF,
                      "every corpus pair differs by >= %.3f m in some feature (closest %.4f m)",
                      (double)CV_MIN_LINF, (double)worstLinf);

                if (worstL2 < CV_MIN_L2 && l2A >= 0) {
                    char ida[128], idb[128];
                    car_corpus_id(l2A, ida, sizeof(ida));
                    car_corpus_id(l2B, idb, sizeof(idb));
                    printf("      closest signature pair '%s' vs '%s': L2 %.4f\n", ida, idb,
                           (double)worstL2);
                }
                check(worstL2 >= CV_MIN_L2,
                      "every corpus pair differs by L2 >= %.2f across the signature"
                      " (closest %.4f)",
                      (double)CV_MIN_L2, (double)worstL2);
            }
        }
        free(maps);
        free(visuals);
    }
}
/* ------------------------------------------------------------------------------------- */
/* Scenario: signature invariants — retired slots, exhaust continuity, independence */
/* ------------------------------------------------------------------------------------- */

/* CAR_SIG_* enum values are private to car_visual.c; look up indices by name instead. */
static int sig_index_of(const char *name)
{
    const int n = car_visual_signature_count();
    for (int i = 0; i < n; i++) {
        if (strcmp(car_visual_signature_component_name(i), name) == 0) return i;
    }
    return -1;
}

static void scenario_signature_invariants(void)
{
    const int n = car_visual_signature_count();
    const int idxSpoke = sig_index_of("spoke_level");
    const int idxRaceDetail = sig_index_of("race_detail_weight");
    const int idxTowFlag = sig_index_of("tow_hook_flag");
    const int idxHoodFlag = sig_index_of("hood_pins_flag");
    const int idxTowDia = sig_index_of("tow_hook_diameter");
    const int idxHoodDia = sig_index_of("hood_pin_diameter");
    check(idxSpoke >= 0 && idxRaceDetail >= 0 && idxTowFlag >= 0 && idxHoodFlag >= 0,
          "retired signature component indices resolve by name");

    /* spoke_level is retired — an invisible feature cannot earn distinctness. Two specs
     * that differ ONLY in wheelInertiaKgM2 must share a bake key and a zero spoke_level. */
    {
        VehicleSpec lo, hi;
        CarVisual vlo, vhi;
        vehicle_spec_set_default(&lo);
        vehicle_spec_set_default(&hi);
        lo.wheelInertiaKgM2 = 0.30f; /* below the lowest spoke threshold */
        hi.wheelInertiaKgM2 = 3.00f; /* above the highest spoke threshold */
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);

        float slo[CAR_SIGNATURE_MAX], shi[CAR_SIGNATURE_MAX];
        car_visual_signature(&vlo, slo, n);
        car_visual_signature(&vhi, shi, n);

        check_near((double)slo[idxSpoke], 0.0, 0.0,
                   "spoke_level is retired to 0 (low inertia)");
        check_near((double)shi[idxSpoke], 0.0, 0.0,
                   "spoke_level is retired to 0 (high inertia)");
        check(car_visual_bake_key(&vlo) == car_visual_bake_key(&vhi),
              "wheelInertiaKgM2 alone does not change the bake key (spokes not drawn)");
        check_near((double)fabsf(slo[idxSpoke] - shi[idxSpoke]), 0.0, 0.0,
                   "retired spoke_level contributes 0 to the signature distance");
    }

    /* exhaustTransition produces area-continuous bore scaling across count thresholds.
     * count × bore² must stay continuous across the 4-cyl (1→2) and 8-cyl (2→4) boundaries. */
    {
        const float cylValues[] = { 3.9f, 4.1f, 7.9f, 8.1f };
        float areas[4];
        for (int i = 0; i < 4; i++) {
            VehicleSpec s;
            CarVisual v;
            vehicle_spec_set_default(&s);
            s.engineCylinders = cylValues[i];
            car_visual_derive(&s, &v);
            const float boreScale = 0.70710678f + 0.29289322f * v.exhaustTransition;
            const float bore = v.exhaustBoreM * boreScale;
            areas[i] = (float)v.exhaustCount * bore * bore;
            check(v.exhaustTransition >= 0.0f && v.exhaustTransition <= 1.0f,
                  "exhaustTransition at %.1f cyl is in [0,1] (got %.3f)", (double)cylValues[i],
                  (double)v.exhaustTransition);
        }
        const float jump4 = fabsf(areas[0] - areas[1]) / (areas[0] + areas[1] + 1e-12f);
        check(jump4 < 0.15f,
              "exhaust area is continuous across the 4-cyl boundary (rel jump %.3f)",
              (double)jump4);
        const float jump8 = fabsf(areas[2] - areas[3]) / (areas[2] + areas[3] + 1e-12f);
        check(jump8 < 0.15f,
              "exhaust area is continuous across the 8-cyl boundary (rel jump %.3f)",
              (double)jump8);
    }

    /* Retired race-detail slots are 0 across the whole corpus, but the rendered geometry
     * (tow-hook/hood-pin diameters) still carries the visible signal where it exists. */
    {
        const int count = car_corpus_count();
        bool allRetiredZero = true;
        bool someRenderedGeometry = false;
        for (int i = 0; i < count; i++) {
            VehicleSpec s;
            CarVisual v;
            float sig[CAR_SIGNATURE_MAX];
            if (!car_corpus_spec(i, &s)) continue;
            car_visual_derive(&s, &v);
            car_visual_signature(&v, sig, n);
            if (sig[idxRaceDetail] != 0.0f || sig[idxTowFlag] != 0.0f ||
                sig[idxHoodFlag] != 0.0f) {
                allRetiredZero = false;
            }
            if (sig[idxTowDia] > 0.0f || sig[idxHoodDia] > 0.0f) {
                someRenderedGeometry = true;
            }
        }
        check(allRetiredZero,
              "race_detail_weight, tow_hook_flag, hood_pins_flag are 0 across the corpus");
        check(someRenderedGeometry,
              "tow-hook/hood-pin rendered geometry still appears in the corpus");
    }
}

/* The render-only scaffolds carry published contracts but no feature yet. These checks are
 * their first consumer, so the headers are compiled and exercised rather than merely shipped,
 * and they assert only what stays true once the real derivations land. */
static void scenario_render_contracts(void)
{
    /* A NULL input returns a zeroed effect state — permanent, not placeholder behaviour. */
    {
        const VehicleVisualEffects zero = vehicle_visual_effects_derive(NULL);
        bool allZero = (zero.brakeLamp01 == 0.0f) && (zero.bodyRollRad == 0.0f) &&
                       (zero.shadowOffsetBodyM.x == 0.0f) && (zero.shadowOffsetBodyM.y == 0.0f);
        for (int i = 0; i < WHEEL_COUNT; i++) {
            if (zero.tireSmoke01[i] != 0.0f) allZero = false;
        }
        check(allZero, "vehicle_visual_effects_derive(NULL) is a zeroed effect state");
    }

    /* A populated input stays bounded and finite. The placeholder satisfies this trivially;
     * the real derivation must satisfy it too, so this check outlives the stub. */
    {
        VehicleEffectInputs in;
        memset(&in, 0, sizeof(in));
        in.brakeInput01 = 1.0f;
        in.filteredLateralAccelerationMps2 = 12.0f;
        for (int i = 0; i < WHEEL_COUNT; i++) {
            in.frictionUsage01[i] = 1.0f;
            in.slipAngleRad[i] = 0.6f;
            in.slipRatio[i] = 0.9f;
            in.surface[i] = SURFACE_ASPHALT;
        }

        const VehicleVisualEffects fx = vehicle_visual_effects_derive(&in);
        bool bounded = (fx.brakeLamp01 >= 0.0f && fx.brakeLamp01 <= 1.0f) &&
                       isfinite(fx.bodyRollRad) && isfinite(fx.shadowOffsetBodyM.x) &&
                       isfinite(fx.shadowOffsetBodyM.y);
        for (int i = 0; i < WHEEL_COUNT; i++) {
            if (!(fx.tireSmoke01[i] >= 0.0f && fx.tireSmoke01[i] <= 1.0f)) bounded = false;
        }
        check(bounded, "derived effects stay bounded and finite for a saturated input");
    }

    /* CarAppearanceSpec's presence bit: a zero-initialised spec means "no explicit seed", so
     * seed 0 is a usable identity rather than a sentinel. */
    {
        const CarAppearanceSpec implicit = { 0 };
        check(!implicit.hasSeed, "a zero-initialised CarAppearanceSpec carries no seed");

        const CarAppearanceSpec explicitZero = { true, 0u };
        check(explicitZero.hasSeed && explicitZero.seed == 0u,
              "seed 0 is a valid explicit identity, distinct from absence");
    }
}

static const TestScenario kAppearanceScenarios[] = {
    { "car-visual", "appearance is a pure, total function of VehicleSpec",
      scenario_car_visual },
    { "render-contracts", "render-only scaffolds honour their published contracts",
      scenario_render_contracts },
    { "corpus", "every corpus vehicle is valid and visibly distinct", scenario_corpus },
    { "signature-invariants", "retired invisible/collinear slots, exhaust area-continuity",
      scenario_signature_invariants },
};

TestScenarioGroup test_appearance_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kAppearanceScenarios;
    group.count = sizeof(kAppearanceScenarios) / sizeof(kAppearanceScenarios[0]);
    return group;
}
