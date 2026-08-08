/*
 * hotreload_harness.c — windowless validation of the Windows hot-reload loader.
 *
 * Drives hotreload_windows.c without opening a raylib window. Confirms:
 *   - the initial module loads
 *   - persistent Game state is platform-owned
 *   - a corrupt candidate is rejected; the previous module stays callable
 *   - game_post_reload does not run for a rejected candidate
 *   - a valid rebuilt candidate swaps in; game_post_reload runs once
 *   - tick / position / wheel speed / RPM / gear / checksum survive the swap
 *   - stale load copies are cleaned up after unload
 *
 * Failed-compile preservation of build/dev/game.dll is exercised by
 * tools/setup/validate_hotreload.sh (the harness itself must not depend on a live
 * interactive drifty.exe).
 */
#ifndef _WIN32
#error Drifty currently supports Windows only.
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"

#define WIN32_LEAN_AND_MEAN
#define NOGDI
#define NOUSER
#define NOMINMAX
#include <windows.h>
#undef near
#undef far

#include "core/config.h"
#include "game/game.h"
#include "platform/hotreload.h"
#include "game/input.h"
#include "platform/timestep.h"

static void fixed_update_adapter(void *ctx, float dt)
{
    game_fixed_update((Game *)ctx, dt);
}

static void run_fixed_ticks(Game *game, int ticks)
{
    for (int i = 0; i < ticks; i++) {
        input_zero(&game->input);
        game->input.throttle = 1.0f;
        game->input.steer = 0.25f;
        (void)timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops, FIXED_DT_S,
                               fixed_update_adapter, game);
    }
}

static int count_load_copies(void)
{
    WIN32_FIND_DATAA find;
    HANDLE search = FindFirstFileA("build\\dev\\game_load_*.dll", &find);
    int count = 0;
    if (search == INVALID_HANDLE_VALUE) return 0;
    do {
        count++;
    } while (FindNextFileA(search, &find));
    FindClose(search);
    return count;
}

static int write_file_bytes(const char *path, const void *data, size_t length)
{
    FILE *f = fopen(path, "wb");
    if (f == NULL) return 0;
    const size_t n = fwrite(data, 1, length, f);
    fclose(f);
    return n == length;
}

static int bump_module_mtime(void)
{
    /* GetFileModTime resolution is one second; ensure the next poll sees a change. */
    Sleep(1100);
    HANDLE file =
        CreateFileA(GAME_MODULE_NAME, FILE_WRITE_ATTRIBUTES, FILE_SHARE_READ | FILE_SHARE_WRITE,
                    NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (file == INVALID_HANDLE_VALUE) return 0;
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);
    const BOOL ok = SetFileTime(file, NULL, NULL, &ft);
    CloseHandle(file);
    return ok ? 1 : 0;
}

int main(void)
{
    SetTraceLogLevel(LOG_WARNING);

    Game *game = (Game *)calloc(1, sizeof(Game));
    if (game == NULL) {
        fprintf(stderr, "harness: calloc failed\n");
        return 1;
    }

    // cppcheck-suppress knownConditionTrueFalse
    if (!Game_LoadModule()) {
        fprintf(stderr, "harness: initial Game_LoadModule failed\n");
        free(game);
        return 1;
    }

    game_init(game);
    run_fixed_ticks(game, 48);

    const uint64_t tick_before = game->sim.tick;
    const Vector2 pos_before = game->vehicle.positionM;
    const uint32_t checksum_before = game->stateChecksum;
    const int reloads_before = game->reloadCount;

    if (tick_before == 0) {
        fprintf(stderr, "harness: expected nonzero tick after warm-up\n");
        Game_UnloadModule();
        free(game);
        return 1;
    }

    /* Keep a full PE backup so the corrupt-candidate path can restore cleanly. */
    if (!CopyFileA(GAME_MODULE_NAME, "build/dev/game_harness_backup.dll", FALSE)) {
        fprintf(stderr, "harness: could not backup %s\n", GAME_MODULE_NAME);
        Game_UnloadModule();
        free(game);
        return 1;
    }

    /* --- corrupt candidate must be rejected -------------------------------------- */
    {
        const unsigned char garbage[] = "NOT_A_PE_DLL_CORRUPT_CANDIDATE";
        if (!write_file_bytes(GAME_MODULE_NAME, garbage, sizeof(garbage) - 1)) {
            fprintf(stderr, "harness: failed to write corrupt candidate\n");
            Game_UnloadModule();
            free(game);
            return 1;
        }
        if (!bump_module_mtime()) {
            fprintf(stderr, "harness: failed to bump mtime for corrupt candidate\n");
            Game_UnloadModule();
            free(game);
            return 1;
        }

        if (Game_MaybeHotReload(game)) {
            fprintf(stderr, "harness: corrupt candidate was accepted\n");
            Game_UnloadModule();
            free(game);
            return 1;
        }
        if (game->reloadCount != reloads_before) {
            fprintf(stderr, "harness: game_post_reload ran for rejected candidate (%d -> %d)\n",
                    reloads_before, game->reloadCount);
            Game_UnloadModule();
            free(game);
            return 1;
        }

        /* Previous module must still be callable. */
        run_fixed_ticks(game, 8);
        if (game->sim.tick <= tick_before) {
            fprintf(stderr, "harness: previous module stopped advancing after reject\n");
            Game_UnloadModule();
            free(game);
            return 1;
        }

        if (!CopyFileA("build/dev/game_harness_backup.dll", GAME_MODULE_NAME, FALSE)) {
            fprintf(stderr, "harness: failed to restore module backup\n");
            Game_UnloadModule();
            free(game);
            return 1;
        }
    }

    /* Publish a fresh, valid candidate. Copying the known-good backup with a new mtime
     * exercises the loader swap without nesting a compiler inside system(). The companion
     * tools/setup/validate_hotreload.sh covers deliberate compile-failure preservation. */
    if (!CopyFileA("build/dev/game_harness_backup.dll", GAME_MODULE_NAME, FALSE)) {
        fprintf(stderr, "harness: failed to publish valid candidate\n");
        Game_UnloadModule();
        free(game);
        return 1;
    }
    if (!bump_module_mtime()) {
        fprintf(stderr, "harness: failed to bump mtime for valid candidate\n");
        Game_UnloadModule();
        free(game);
        return 1;
    }

    const uint64_t tick_pre_swap = game->sim.tick;
    const Vector2 pos_pre_swap = game->vehicle.positionM;
    const float rear_omega_pre_swap = game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
    const float engine_rpm_pre_swap = game->vehicle.engineRpm;
    const int gear_pre_swap = game->vehicle.selectedGear;
    const uint32_t checksum_pre_swap = game->stateChecksum;
    const int reloads_pre_swap = game->reloadCount;

    if (!Game_MaybeHotReload(game)) {
        fprintf(stderr, "harness: valid candidate was not swapped in\n");
        Game_UnloadModule();
        free(game);
        return 1;
    }
    if (game->reloadCount != reloads_pre_swap + 1) {
        fprintf(stderr, "harness: expected reloadCount %d after swap, got %d\n",
                reloads_pre_swap + 1, game->reloadCount);
        Game_UnloadModule();
        free(game);
        return 1;
    }
    if (game->sim.tick != tick_pre_swap || game->vehicle.positionM.x != pos_pre_swap.x ||
        game->vehicle.positionM.y != pos_pre_swap.y ||
        game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS != rear_omega_pre_swap ||
        game->vehicle.engineRpm != engine_rpm_pre_swap ||
        game->vehicle.selectedGear != gear_pre_swap ||
        game->stateChecksum != checksum_pre_swap) {
        fprintf(stderr, "harness: persistent state did not survive swap\n");
        Game_UnloadModule();
        free(game);
        return 1;
    }

    /* New code path still executes. */
    run_fixed_ticks(game, 8);
    if (game->sim.tick <= tick_pre_swap) {
        fprintf(stderr, "harness: swapped module did not advance simulation\n");
        Game_UnloadModule();
        free(game);
        return 1;
    }

    Game_UnloadModule();
    const int leftover = count_load_copies();
    DeleteFileA("build/game_harness_backup.dll");

    if (leftover != 0) {
        fprintf(stderr, "harness: %d stale build/dev/game_load_*.dll copies remain\n",
                leftover);
        free(game);
        return 1;
    }

    printf(
        "hotreload harness: ok (warm tick=%llu checksum=%08x pos=(%.3f,%.3f); "
        "reject preserved module; swap preserved tick=%llu checksum=%08x; post_reload once)\n",
        (unsigned long long)tick_before, checksum_before, (double)pos_before.x,
        (double)pos_before.y, (unsigned long long)tick_pre_swap, checksum_pre_swap);

    free(game);
    return 0;
}
