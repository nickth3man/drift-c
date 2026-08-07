/*
 * render_hud.c — screen-space presentation: the diagnostics readout, the arcade HUD, and
 * the full-screen state overlays, plus the palette and text helpers they share. Compiled
 * to an empty translation unit under DRIFTY_HEADLESS.
 */
#if !defined(DRIFTY_HEADLESS)

#include "render/render_internal.h"

#include <math.h>

#include "core/math_utils.h"
#include "physics/tire.h"
#include "physics/vehicle.h"
#include "core/units.h"

/* ---- presentation palette, type scale, and screen-space helpers -----------------------
 *
 * Arcade-drift palette (documented for the design review):
 *   COL_ACCENT       hot gold    - score, combo, DRIFT! callouts, the car nose marker
 *   COL_ACCENT_WARM  warm orange - NEW BEST flash and other "payoff" moments
 *   COL_COOL         steel cyan  - secondary info (best score, rpm bar fill)
 *   COL_TEXT         near-white  - primary HUD text
 *   COL_TEXT_DIM     slate       - secondary and hint text
 *   COL_PANEL        translucent charcoal - backing panels behind HUD clusters
 *   COL_PANEL_EDGE   faint white          - panel outline, separates panel from track
 *   COL_DIM_SCREEN   heavy translucent charcoal - full-screen dim behind overlays
 *
 * Type scale (one scale, used everywhere below):
 *   title 64 | overlay heading 40-48 | results figure 56 | cluster figure 34-46 |
 *   body 18-20 | labels/hints 14-16 | micro 12.
 */
const Color COL_ACCENT = { 255, 198, 64, 255 };
const Color COL_ACCENT_WARM = { 255, 120, 72, 255 };
const Color COL_COOL = { 110, 205, 235, 255 };
const Color COL_TEXT = { 236, 238, 242, 255 };
const Color COL_TEXT_DIM = { 152, 158, 170, 255 };
const Color COL_PANEL = { 12, 14, 18, 170 };
const Color COL_PANEL_EDGE = { 255, 255, 255, 26 };
const Color COL_DIM_SCREEN = { 8, 10, 14, 185 };

/* The car's own palette used to live here as COL_CAR_BODY / COL_CAR_OUTLINE / COL_CAR_CABIN /
 * COL_TIRE / COL_RIM, next to a hardcoded 4.2 x 1.82 m box. Both are gone: the vehicle's
 * colours and its geometry are now derived in src/render/car_visual.c and rasterized by
 * src/render/car_visual_raster.c, and this file only uploads and draws what they produce. Adding a
 * styling decision back here would put it where drifty_tests cannot reach it. */

static void draw_text_centered(const char *text, int y, int fontSize, Color color)
{
    DrawText(text, (SCREEN_W - MeasureText(text, fontSize)) / 2, y, fontSize, color);
}

static void draw_text_centered_shadow(const char *text, int y, int fontSize, Color color)
{
    const int x = (SCREEN_W - MeasureText(text, fontSize)) / 2;
    DrawText(text, x + 3, y + 3, fontSize, (Color){ 0, 0, 0, 170 });
    DrawText(text, x, y, fontSize, color);
}

static void draw_hud_panel(Rectangle rec)
{
    DrawRectangleRounded(rec, 0.16f, 6, COL_PANEL);
    DrawRectangleRoundedLines(rec, 0.16f, 6, COL_PANEL_EDGE);
}

/* Slow pulse between two alpha levels, for "press a key" prompts.
 * Render-only time source; nothing here feeds the simulation. */
static unsigned char pulse_alpha(float cyclesPerSecond, unsigned char lo, unsigned char hi)
{
    const float s = 0.5f + 0.5f * sinf((float)GetTime() * 6.2831853f * cyclesPerSecond);
    return (unsigned char)(lo + (unsigned char)((float)(hi - lo) * s));
}

static const char *gear_label(int selectedGear);

/* ---- full-screen overlays (STATE_MENU / STATE_PAUSED / STATE_RESULTS) ----------------
 * Pure screen space: called after EndMode2D so the camera transform never touches them.
 * Copy is deliberately short and direct; wording is up for review.
 */
static void draw_overlay_menu(const Game *game)
{
    (void)game;
    DrawRectangle(0, 0, SCREEN_W, SCREEN_H, COL_DIM_SCREEN);

    draw_text_centered_shadow("DRIFTY", 226, 64, COL_ACCENT);
    draw_text_centered("a tiny top-down drift sandbox", 306, 20, COL_TEXT_DIM);

    draw_text_centered(
        "PRESS P TO START", 414, 24,
        (Color){ COL_TEXT.r, COL_TEXT.g, COL_TEXT.b, pulse_alpha(0.6f, 90, 255) });
    draw_text_centered("W/S throttle & brake    A/D steer    SPACE handbrake    "
                       "Q/E shift    P pause    R reset",
                       466, 16, COL_TEXT_DIM);
}

static void draw_overlay_paused(const Game *game)
{
    (void)game;
    DrawRectangle(0, 0, SCREEN_W, SCREEN_H,
                  (Color){ COL_DIM_SCREEN.r, COL_DIM_SCREEN.g, COL_DIM_SCREEN.b, 150 });
    draw_text_centered_shadow("PAUSED", 300, 48, COL_TEXT);
    draw_text_centered("P resume    R reset", 372, 18, COL_TEXT_DIM);
}

static void draw_overlay_results(const Game *game)
{
    DrawRectangle(0, 0, SCREEN_W, SCREEN_H, COL_DIM_SCREEN);

    draw_text_centered_shadow("RUN COMPLETE", 196, 40, COL_TEXT);

    /* Lap time is the primary result now. */
    const int mins = (int)(game->track.lastLapTimeS / 60.0f);
    const float secs = game->track.lastLapTimeS - (float)mins * 60.0f;
    draw_text_centered_shadow(TextFormat("%d:%05.2f", mins, (double)secs), 286, 56, COL_ACCENT);
    draw_text_centered("LAP TIME", 352, 16, COL_TEXT_DIM);

    draw_text_centered("P drive again    R menu", 504, 18, COL_TEXT_DIM);
}

/* ---- arcade HUD clusters -------------------------------------------------------------
 * Two clusters with clear hierarchy, each on a translucent panel so it reads against
 * any track background:
 *   speed  - bottom-left: km/h large, gear, rpm bar. The most-glanced readout.
 *   lap    - top-center:  lap count, running timer, checkpoint progress.
 */
void render_hud_draw_arcade(const Game *game)
{
    /* ---- speed cluster (bottom-left) ---- */
    {
        const Rectangle panel = { 18.0f, SCREEN_H - 168.0f, 244.0f, 140.0f };
        draw_hud_panel(panel);

        const float kmh = game->derived.speedMps * 3.6f;
        const char *kmhText = TextFormat("%.0f", (double)kmh);
        DrawText(kmhText, (int)panel.x + 16, (int)panel.y + 12, 46, COL_TEXT);
        DrawText("KM/H", (int)panel.x + 20 + MeasureText(kmhText, 46), (int)panel.y + 40, 16,
                 COL_TEXT_DIM);

        const char *modeLabel = game->autoTrans.enabled ? "AUTO " : "";
        DrawText(TextFormat("%sGEAR %s", modeLabel, gear_label(game->vehicle.selectedGear)),
                 (int)panel.x + 16, (int)panel.y + 66, 18, COL_TEXT);

        /* RPM bar: cool cyan, flipping to accent gold near the redline. */
        const float idleRpm = game->spec.engineIdleRpm;
        const float redlineRpm = game->spec.engineRedlineRpm;
        const float rpmFrac =
            clampf((game->vehicle.engineRpm - idleRpm) / (redlineRpm - idleRpm), 0.0f, 1.0f);
        const Rectangle barBg = { panel.x + 16.0f, panel.y + 98.0f, panel.width - 32.0f,
                                  10.0f };
        DrawRectangleRec(barBg, (Color){ 255, 255, 255, 22 });
        DrawRectangleRec((Rectangle){ barBg.x, barBg.y, barBg.width * rpmFrac, barBg.height },
                         (rpmFrac > 0.85f) ? COL_ACCENT : COL_COOL);
        DrawText(TextFormat("%.0f RPM", (double)game->vehicle.engineRpm), (int)panel.x + 16,
                 (int)panel.y + 114, 12, COL_TEXT_DIM);
    }

    /* ---- lap cluster (top-center) ---- */
    {
        const Rectangle panel = { (SCREEN_W - 360.0f) * 0.5f, 16.0f, 360.0f, 56.0f };
        draw_hud_panel(panel);

        int shownLap = game->track.lap + 1;
        if (shownLap > RESULTS_TARGET_LAPS) shownLap = RESULTS_TARGET_LAPS;
        DrawText(TextFormat("LAP %d/%d", shownLap, RESULTS_TARGET_LAPS), (int)panel.x + 16,
                 (int)panel.y + 18, 18, COL_TEXT);

        const int minutes = (int)(game->track.lapTimerS / 60.0f);
        const float seconds = game->track.lapTimerS - (float)minutes * 60.0f;
        const char *timerText = TextFormat("%d:%05.2f", minutes, (double)seconds);
        DrawText(timerText,
                 (int)(panel.x + (panel.width - (float)MeasureText(timerText, 22)) * 0.5f),
                 (int)panel.y + 16, 22, COL_TEXT);

        const char *cpText =
            TextFormat("CP %d/%d", game->track.nextCheckpoint, game->track.checkpointCount);
        DrawText(cpText, (int)(panel.x + panel.width) - 16 - MeasureText(cpText, 18),
                 (int)panel.y + 18, 18, COL_TEXT_DIM);
    }

    /* Hot-reload notice, top-left: preserves the information the old HUD line carried. */
    if (game->reloadFlashTimerS > 0.0f) {
        DrawText("module reloaded - state preserved", 18, 18, 14, COL_ACCENT);
    }

    DrawText("W throttle  S brake  Space handbrake  Q/E shift  A/D steer  R reset  "
             "F1 diagnostics  F2 physics lab",
             (SCREEN_W - MeasureText("W throttle  S brake  Space handbrake  Q/E shift  "
                                     "A/D steer  R reset  F1 diagnostics  F2 physics lab",
                                     14)) /
                 2,
             SCREEN_H - 26, 14, COL_TEXT_DIM);
}

static void hud_line(int x, int *y, const char *text, Color color)
{
    DrawText(text, x, *y, 16, color);
    *y += 19;
}

static Vector2 plot_point(Rectangle bounds, float x, float xMin, float xMax, float y,
                          float yMin, float yMax)
{
    return (Vector2){ bounds.x + (x - xMin) / (xMax - xMin) * bounds.width,
                      bounds.y + bounds.height - (y - yMin) / (yMax - yMin) * bounds.height };
}

static void draw_curve_axes(Rectangle bounds, float xMin, float xMax, float yMin, float yMax)
{
    DrawRectangleRec(bounds, (Color){ 16, 18, 22, 235 });
    DrawRectangleLinesEx(bounds, 1.0f, (Color){ 100, 106, 116, 255 });
    DrawLineV(plot_point(bounds, xMin, xMin, xMax, 0.0f, yMin, yMax),
              plot_point(bounds, xMax, xMin, xMax, 0.0f, yMin, yMax),
              (Color){ 80, 84, 92, 255 });
    DrawLineV(plot_point(bounds, 0.0f, xMin, xMax, yMin, yMin, yMax),
              plot_point(bounds, 0.0f, xMin, xMax, yMax, yMin, yMax),
              (Color){ 80, 84, 92, 255 });
}

static void draw_tire_curve_panel(const Game *game)
{
    const Rectangle lateral = { SCREEN_W - 390.0f, 26.0f, 365.0f, 145.0f };
    const Rectangle longitudinal = { SCREEN_W - 390.0f, 205.0f, 365.0f, 145.0f };
    const float latMin = -0.55f;
    const float latMax = 0.55f;
    const float forceMin = -1.4f;
    const float forceMax = 1.4f;
    draw_curve_axes(lateral, latMin, latMax, forceMin, forceMax);
    DrawText("LATERAL  normalized force / wheel load", (int)lateral.x, (int)lateral.y - 19, 14,
             RAYWHITE);
    DrawText("front", (int)lateral.x + 6, (int)lateral.y + 5, 12, ORANGE);
    DrawText("rear", (int)lateral.x + 52, (int)lateral.y + 5, 12, SKYBLUE);
    Vector2 prevFront = { 0.0f, 0.0f };
    Vector2 prevRear = { 0.0f, 0.0f };
    for (int i = 0; i <= 120; i++) {
        const float slip = lerpf(latMin, latMax, (float)i / 120.0f);
        const float front =
            -game->spec.tireMuLatFront *
            tire_normalized_curve(game->spec.tireBLatFront, game->spec.tireCLatFront, slip);
        const float rear =
            -game->spec.tireMuLatRear *
            tire_normalized_curve(game->spec.tireBLatRear, game->spec.tireCLatRear, slip);
        const Vector2 pFront =
            plot_point(lateral, slip, latMin, latMax, front, forceMin, forceMax);
        const Vector2 pRear =
            plot_point(lateral, slip, latMin, latMax, rear, forceMin, forceMax);
        if (i > 0) {
            DrawLineV(prevFront, pFront, ORANGE);
            DrawLineV(prevRear, pRear, SKYBLUE);
        }
        prevFront = pFront;
        prevRear = pRear;
    }
    const float currentFront =
        -game->spec.tireMuLatFront * tire_normalized_curve(game->spec.tireBLatFront,
                                                           game->spec.tireCLatFront,
                                                           game->derived.frontSlipAngleRad);
    const float currentRear =
        -game->spec.tireMuLatRear * tire_normalized_curve(game->spec.tireBLatRear,
                                                          game->spec.tireCLatRear,
                                                          game->derived.rearSlipAngleRad);
    DrawCircleV(plot_point(lateral, game->derived.frontSlipAngleRad, latMin, latMax,
                           currentFront, forceMin, forceMax),
                4.0f, ORANGE);
    DrawCircleV(plot_point(lateral, game->derived.rearSlipAngleRad, latMin, latMax, currentRear,
                           forceMin, forceMax),
                4.0f, SKYBLUE);

    const float longMin = -1.25f;
    const float longMax = 1.25f;
    draw_curve_axes(longitudinal, longMin, longMax, -1.1f, 1.1f);
    DrawText("LONGITUDINAL  normalized force / wheel load", (int)longitudinal.x,
             (int)longitudinal.y - 19, 14, RAYWHITE);
    Vector2 previous = { 0.0f, 0.0f };
    for (int i = 0; i <= 120; i++) {
        const float slip = lerpf(longMin, longMax, (float)i / 120.0f);
        const float force =
            game->spec.tireMuLongScale *
            tire_normalized_curve(game->spec.tireBLong, game->spec.tireCLong, slip);
        const Vector2 point =
            plot_point(longitudinal, slip, longMin, longMax, force, -1.1f, 1.1f);
        if (i > 0) DrawLineV(previous, point, LIME);
        previous = point;
    }
    const float rearSlip = game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio;
    const float rearLong =
        game->spec.tireMuLongScale *
        tire_normalized_curve(game->spec.tireBLong, game->spec.tireCLong, rearSlip);
    DrawCircleV(plot_point(longitudinal, rearSlip, longMin, longMax, rearLong, -1.1f, 1.1f),
                4.0f, YELLOW);
    DrawText("zero axes; curve peaks are the configured friction references",
             (int)longitudinal.x, (int)longitudinal.y + (int)longitudinal.height + 5, 11,
             (Color){ 155, 160, 170, 255 });
}

static const char *gear_label(int selectedGear)
{
    if (selectedGear < 0) return "R";
    if (selectedGear == 0) return "N";
    static const char *const labels[MAX_GEARS] = { "1", "2", "3", "4", "5", "6", "7", "8" };
    if (selectedGear <= MAX_GEARS) return labels[selectedGear - 1];
    return "?";
}

/* ---- raw physics diagnostics (F1) -------------------------------------------------
 * The development readout. Everything here draws only when the debug overlay is on; the
 * always-on presentation is the arcade HUD. Lines are unchanged from the previous
 * always-on stack, just gated. */
void render_hud_draw_diagnostics(const Game *game, float alpha)
{
    if (game->debugOverlay) {
        const Color label = (Color){ 235, 235, 235, 255 };
        const Color dim = (Color){ 155, 160, 170, 255 };
        int y = 12;
        hud_line(14, &y, "DRIFTY diagnostics", label);
        hud_line(14, &y,
                 TextFormat("pos (%+.2f,%+.2f) m  heading %+.3f rad",
                            (double)game->vehicle.positionM.x,
                            (double)game->vehicle.positionM.y,
                            (double)game->vehicle.headingRad),
                 dim);
        hud_line(14, &y,
                 TextFormat("vx %+.3f m/s  vy %+.3f m/s  speed %.3f m/s  yaw %+.3f rad/s",
                            (double)game->vehicle.velocityLongitudinalMps,
                            (double)game->vehicle.velocityLateralMps,
                            (double)game->derived.speedMps, (double)game->vehicle.yawRateRadS),
                 label);
        hud_line(14, &y,
                 TextFormat("steer %+.3f  slip F/R %+.3f / %+.3f rad  sideslip %+.3f",
                            (double)game->vehicle.frontRoadWheelAngleRad,
                            (double)game->derived.frontSlipAngleRad,
                            (double)game->derived.rearSlipAngleRad,
                            (double)game->derived.bodySideslipRad),
                 dim);
        hud_line(14, &y,
                 TextFormat("gear %s  engine %.0f rpm  rear drive %+.1f Nm",
                            gear_label(game->vehicle.selectedGear),
                            (double)game->vehicle.engineRpm,
                            (double)game->derived.drivelineTorqueNm),
                 label);
        hud_line(14, &y,
                 TextFormat("omega FL/FR/R %.2f / %.2f / %.2f rad/s  surface R %+.2f m/s",
                            (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS,
                            (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS,
                            (double)game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS,
                            (double)(game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS *
                                     vehicle_wheel_radius_m(&game->spec, WHEEL_REAR_LEFT))),
                 dim);
        hud_line(14, &y,
                 TextFormat("kappa FL/FR/RL/RR %+.3f %+.3f %+.3f %+.3f",
                            (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio,
                            (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].slipRatio,
                            (double)game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio,
                            (double)game->vehicle.wheels[WHEEL_REAR_RIGHT].slipRatio),
                 dim);
        hud_line(14, &y,
                 TextFormat(
                     "load F/R %.1f / %.1f N  transfer %+.1f N  lateral F/R %+.1f / %+.1f N",
                     (double)game->derived.normalLoadFrontN,
                     (double)game->derived.normalLoadRearN, (double)game->derived.loadTransferN,
                     (double)game->derived.frontLateralForceN,
                     (double)game->derived.rearLateralForceN),
                 dim);
        hud_line(14, &y,
                 TextFormat("body force (%+.1f,%+.1f) N  yaw torque %+.1f Nm  blend %.3f",
                            (double)game->derived.totalBodyForceN.x,
                            (double)game->derived.totalBodyForceN.y,
                            (double)game->derived.totalYawTorqueNm,
                            (double)game->derived.lowSpeedBlend),
                 dim);
        hud_line(14, &y,
                 TextFormat("substeps %d  backlog %d  alpha %.3f  tick %llu  checksum %08x",
                            game->lastSubstepCount, game->physicsBacklogDrops, (double)alpha,
                            (unsigned long long)game->sim.tick, game->stateChecksum),
                 label);
        hud_line(14, &y,
                 TextFormat("reloads %d%s  gear %s  render %.1f px/m", game->reloadCount,
                            game->reloadFlashTimerS > 0.0f ? " (state preserved)" : "",
                            gear_label(game->vehicle.selectedGear),
                            (double)game->renderPixelsPerMeter),
                 dim);
        hud_line(14, &y,
                 TextFormat("Lap: %d  Timer: %d:%05.2f  Checkpoint: %d / %d", game->track.lap,
                            (int)(game->track.lapTimerS / 60.0f),
                            (double)(game->track.lapTimerS -
                                     (float)(int)(game->track.lapTimerS / 60.0f) * 60.0f),
                            game->track.nextCheckpoint, game->track.checkpointCount),
                 label);
        {
            hud_line(
                14, &y,
                TextFormat("pure Fx F/R %+.0f/%+.0f  pure Fy F/R %+.0f/%+.0f N",
                           (double)(game->derived.pureLongitudinalForceN[WHEEL_FRONT_LEFT] +
                                    game->derived.pureLongitudinalForceN[WHEEL_FRONT_RIGHT]),
                           (double)(game->derived.pureLongitudinalForceN[WHEEL_REAR_LEFT] +
                                    game->derived.pureLongitudinalForceN[WHEEL_REAR_RIGHT]),
                           (double)(game->derived.pureLateralForceN[WHEEL_FRONT_LEFT] +
                                    game->derived.pureLateralForceN[WHEEL_FRONT_RIGHT]),
                           (double)(game->derived.pureLateralForceN[WHEEL_REAR_LEFT] +
                                    game->derived.pureLateralForceN[WHEEL_REAR_RIGHT])),
                dim);
            hud_line(
                14, &y,
                TextFormat("limited Fx F/R %+.0f/%+.0f  limited Fy F/R %+.0f/%+.0f N",
                           (double)(game->vehicle.wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
                                    game->vehicle.wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN),
                           (double)(game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                                    game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN),
                           (double)game->derived.frontLateralForceN,
                           (double)game->derived.rearLateralForceN),
                dim);
            hud_line(14, &y,
                     TextFormat("usage FL/FR/RL/RR %.2f %.2f %.2f %.2f  lock %d%d%d%d",
                                (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].frictionUsage,
                                (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].frictionUsage,
                                (double)game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage,
                                (double)game->vehicle.wheels[WHEEL_REAR_RIGHT].frictionUsage,
                                game->vehicle.wheels[WHEEL_FRONT_LEFT].locked,
                                game->vehicle.wheels[WHEEL_FRONT_RIGHT].locked,
                                game->vehicle.wheels[WHEEL_REAR_LEFT].locked,
                                game->vehicle.wheels[WHEEL_REAR_RIGHT].locked),
                     dim);
            hud_line(14, &y,
                     TextFormat("brake F/R %.0f/%.0f Nm  handbrake R %.0f Nm",
                                (double)(game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
                                         game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT]),
                                (double)(game->derived.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
                                         game->derived.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT]),
                                (double)(game->derived.handbrakeTorqueNm[WHEEL_REAR_LEFT] +
                                         game->derived.handbrakeTorqueNm[WHEEL_REAR_RIGHT])),
                     dim);
            hud_line(14, &y,
                     TextFormat("ax prev/filt/solved %+.2f/%+.2f/%+.2f m/s^2  "
                                "drag %.0f N  rolling %.0f N",
                                (double)game->derived.previousLongAccelMps2,
                                (double)game->derived.filteredLongAccelMps2,
                                (double)game->derived.solvedLongAccelMps2,
                                (double)game->derived.aeroDragMagnitudeN,
                                (double)game->derived.rollingResistanceMagnitudeN),
                     dim);
            draw_tire_curve_panel(game);
        }
    }
}

void render_hud_draw_state_overlay(const Game *game)
{
    switch (game->state) {
        case STATE_MENU: draw_overlay_menu(game); break;
        case STATE_PAUSED: draw_overlay_paused(game); break;
        case STATE_RESULTS: draw_overlay_results(game); break;
        default: break;
    }
}

#endif /* !DRIFTY_HEADLESS */
