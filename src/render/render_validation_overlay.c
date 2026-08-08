/*
 * render_validation_overlay.c — screen-space diagnostic overlay for offline video capture.
 *
 * Drawn at native resolution after the world and HUD blit, so every frame of the generated MP4
 * carries the car identity, elapsed time, checkpoint progress, speed, running pass/fail status,
 * and input bar indicators.
 *
 * Compiled to a clean no-op under DRIFTY_HEADLESS.
 */
#if !defined(DRIFTY_HEADLESS)

#include "render/render_internal.h"

#include <stdio.h>
#include <string.h>

#include "raylib.h"
#include "core/units.h"

/* The staged overlay. Module-static rather than a field on Game because ValidationOverlayData
 * holds string pointers, and nothing reachable from Game may point into module memory
 * (game.h). Losing the staging across a hot reload costs one frame of overlay and nothing
 * else; the offline capture that uses it never reloads. */
static ValidationOverlayData s_staged;
static bool s_stagedValid;
static char s_stagedCarId[64];
static char s_stagedCarDisplayName[96];

static void copy_bounded(char *dst, size_t cap, const char *src)
{
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    const size_t n = strlen(src);
    const size_t take = (n < cap - 1) ? n : cap - 1;
    memcpy(dst, src, take);
    dst[take] = '\0';
}

GAME_API void render_set_validation_overlay(const ValidationOverlayData *data)
{
    if (data == NULL) {
        s_stagedValid = false;
        return;
    }

    s_staged = *data;
    copy_bounded(s_stagedCarId, sizeof(s_stagedCarId), data->carId);
    copy_bounded(s_stagedCarDisplayName, sizeof(s_stagedCarDisplayName), data->carDisplayName);
    s_staged.carId = s_stagedCarId;
    s_staged.carDisplayName = s_stagedCarDisplayName;
    s_stagedValid = true;
}

void render_draw_staged_validation_overlay(void)
{
    if (!s_stagedValid) return;
    const ValidationOverlayData *data = &s_staged;

    /* Fixed 1280x720 canvas dimensions used by the production capture pipeline. */
    const int screenW = GetScreenWidth();
    const int screenH = GetScreenHeight();

    const Color colBg = { 12, 14, 18, 200 };
    const Color colBorder = { 255, 255, 255, 40 };
    const Color colText = { 236, 238, 242, 255 };
    const Color colSub = { 160, 168, 180, 255 };
    const Color colPass = { 80, 220, 120, 255 };
    const Color colFail = { 240, 80, 80, 255 };

    /* --- Top-left panel: Run identity and state --- */
    const Rectangle topPanel = { 20.0f, 20.0f, 420.0f, 110.0f };
    DrawRectangleRounded(topPanel, 0.12f, 6, colBg);
    DrawRectangleRoundedLines(topPanel, 0.12f, 6, colBorder);

    char titleBuf[128];
    snprintf(titleBuf, sizeof(titleBuf), "[%s] %s", data->carId != NULL ? data->carId : "car",
             data->carDisplayName != NULL ? data->carDisplayName : "");
    DrawText(titleBuf, 35, 32, 20, colText);
    const double kmh = (double)data->speedMps * 3.6;
    char statsBuf[128];
    snprintf(statsBuf, sizeof(statsBuf), "TIME: %.2f s   CP %d / %d   %.1f km/h",
             (double)data->elapsedS, data->checkpointIndex, data->checkpointTotal, kmh);
    DrawText(statsBuf, 35, 60, 16, colSub);

    const char *stateStr = "OUT-LAP";
    if (data->lapState == 1)
        stateStr = "TIMED LAP";
    else if (data->lapState == 2)
        stateStr = "COMPLETE";
    else if (data->lapState == 3)
        stateStr = "FAILED";

    char statusBuf[128];
    snprintf(statusBuf, sizeof(statusBuf), "STATE: %s", stateStr);
    DrawText(statusBuf, 35, 84, 16, colSub);

    const char *passStr = data->isPassing ? "PASS" : "FAIL";
    const Color passCol = data->isPassing ? colPass : colFail;
    DrawText(passStr, 340, 84, 18, passCol);

    /* --- Bottom-left panel: AI Driver Inputs --- */
    const Rectangle botPanel = { 20.0f, (float)(screenH - 120), 320.0f, 90.0f };
    DrawRectangleRounded(botPanel, 0.12f, 6, colBg);
    DrawRectangleRoundedLines(botPanel, 0.12f, 6, colBorder);

    /* Steer bar: centered at x = 180 */
    DrawText("STEER", 35, screenH - 110, 14, colSub);
    const int steerCenterX = 180;
    const int steerBarY = screenH - 108;
    const int steerWidth = 100;
    DrawRectangle(steerCenterX - steerWidth / 2, steerBarY, steerWidth, 12,
                  (Color){ 40, 44, 52, 255 });
    DrawRectangleLines(steerCenterX - steerWidth / 2, steerBarY, steerWidth, 12, colBorder);
    const float steerClamped = (data->steerInput < -1.0f)
                                   ? -1.0f
                                   : (data->steerInput > 1.0f ? 1.0f : data->steerInput);
    const int fillW = (int)(fabsf(steerClamped) * (float)(steerWidth / 2));
    if (steerClamped > 0.0f) {
        DrawRectangle(steerCenterX, steerBarY + 1, fillW, 10, colSub);
    } else if (steerClamped < 0.0f) {
        DrawRectangle(steerCenterX - fillW, steerBarY + 1, fillW, 10, colSub);
    }

    /* Throttle bar */
    DrawText("THROTTLE", 35, screenH - 85, 14, colSub);
    const int barX = 130;
    const int barW = 180;
    DrawRectangle(barX, screenH - 83, barW, 12, (Color){ 40, 44, 52, 255 });
    DrawRectangleLines(barX, screenH - 83, barW, 12, colBorder);
    const float thrClamped = (data->throttleInput < 0.0f)
                                 ? 0.0f
                                 : (data->throttleInput > 1.0f ? 1.0f : data->throttleInput);
    DrawRectangle(barX, screenH - 82, (int)(thrClamped * (float)barW), 10, colPass);

    /* Brake bar */
    DrawText("BRAKE", 35, screenH - 60, 14, colSub);
    DrawRectangle(barX, screenH - 58, barW, 12, (Color){ 40, 44, 52, 255 });
    DrawRectangleLines(barX, screenH - 58, barW, 12, colBorder);
    const float brkClamped =
        (data->brakeInput < 0.0f) ? 0.0f : (data->brakeInput > 1.0f ? 1.0f : data->brakeInput);
    DrawRectangle(barX, screenH - 57, (int)(brkClamped * (float)barW), 10, colFail);

    (void)screenW;
}

#else

#include "render/render.h"

void render_set_validation_overlay(const ValidationOverlayData *data)
{
    (void)data;
}

#endif /* !DRIFTY_HEADLESS */
