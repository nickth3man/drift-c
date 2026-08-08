/*
 * main.c — platform layer (Windows only).
 *
 * Owns the window, the raylib context, the Game allocation, and the fixed-timestep loop.
 * It is not hot-reloadable: editing this file requires restarting drifty.exe.
 *
 * The Game block is allocated here, once, and handed to the game module by pointer on every
 * entry point. That is what lets the module be swapped while the game keeps its state.
 *
 * Normal interactive runs loop until the window closes. Pass --smoke-test for a bounded
 * run that exercises the Phase 3 vehicle and debug HUD, writes
 * artifacts/screenshots/phase3_smoke.png,
 * and exits without leaving a background process.
 *
 * Two other bounded modes exist for tooling:
 *
 *   --capture-scene NAME [--width W] [--height H] [--seed N] [--output PATH] [--ticks N]
 *       Steps the simulation with an exact fixed dt (never GetFrameTime), draws one frame,
 *       writes a PNG, and exits. Frame-rate independence is the point: the same command
 *       produces the same image, which is what makes screenshot comparison meaningful.
 *
 *   --list-scenes
 *       Prints the capture scene names and exits.
 *
 * Both terminate on their own. Nothing in this file ever waits for a developer.
 */
#ifndef _WIN32
#error Drifty currently supports Windows only.
#endif

#include <direct.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "platform/build_info.h"

#include "raylib.h"

#include "core/config.h"
#include "dev/dev_scenario.h"
#include "dev/dev_replay.h"
#include "game/ai_driver.h"
#include "game/car_roster.h"
#include "game/game.h"
#include "game/input.h"
#include "game/run_report.h"
#include "game/validation_metrics.h"
#include "platform/hotreload.h"
#include "platform/timestep.h"
#include "render/render.h"

/* Bounded smoke-test duration: enough frames for the fixed-step loop and HUD to settle. */
#define SMOKE_TEST_FRAMES 120

/* The smoke test's evidence image. It is run evidence, not telemetry, so it lives beside the
 * other captures under artifacts/ rather than in the CSV output directory. */
#define SMOKE_TEST_SCREENSHOT "artifacts/screenshots/phase3_smoke.png"

/* --------------------------------------------------------------------- capture scenes -- */

typedef struct {
    const char *name;
    const char *scenario; /* dev_scenario key driving the sim before the capture */
    int ticks;            /* fixed ticks to run before drawing */
    bool debugOverlay;
    bool lab;
    int scopePreset;
    bool showLoads;
    bool showResistance;
} CaptureScene;

static const CaptureScene g_captureScenes[] = {
    { "debug_overlay", "accel", 240, true, false, DEV_SCOPE_PRESET_HANDLING, false, false },
    { "tire_curves", "skidpad", 600, true, false, DEV_SCOPE_PRESET_GRIP, false, false },
    { "drift_hud", "handbrake-entry", 500, true, false, DEV_SCOPE_PRESET_HANDLING, false,
      false },
    { "physics_lab", "skidpad", 480, false, true, DEV_SCOPE_PRESET_HANDLING, false, false },
    { "accel_load", "accel-load", 600, false, true, DEV_SCOPE_PRESET_LOAD, true, true },
    { "brake_load", "brake-load", 760, false, true, DEV_SCOPE_PRESET_LOAD, true, true },
    { "skidpad_p3", "skidpad", 1500, false, true, DEV_SCOPE_PRESET_GRIP, true, true },
    { "lift_off", "lift-off", 850, false, true, DEV_SCOPE_PRESET_LOAD, true, true },
    { "transition_p3", "transition", 900, false, true, DEV_SCOPE_PRESET_GRIP, true, true },
    { "catchable", "catchable-drift", 820, false, true, DEV_SCOPE_PRESET_GRIP, true, true },
};

#define CAPTURE_SCENE_COUNT ((int)(sizeof(g_captureScenes) / sizeof(g_captureScenes[0])))

static const CaptureScene *find_capture_scene(const char *name)
{
    for (int i = 0; i < CAPTURE_SCENE_COUNT; i++) {
        if (strcmp(g_captureScenes[i].name, name) == 0) return &g_captureScenes[i];
    }
    return NULL;
}

/* Create every directory component of path, so --output artifacts/screenshots/x.png works on
 * a clean checkout. Failure is not fatal: raylib reports the export failure itself. */
static void ensure_parent_directory(const char *path)
{
    char buffer[512];
    snprintf(buffer, sizeof(buffer), "%s", path);

    char *lastSeparator = NULL;
    for (char *c = buffer; *c != '\0'; c++) {
        if (*c == '/' || *c == '\\') lastSeparator = c;
    }
    if (lastSeparator == NULL) return;
    *lastSeparator = '\0';

    /* Walk the path creating one level at a time; EEXIST is the normal case. */
    for (char *c = buffer;; c++) {
        const bool atSeparator = (*c == '/' || *c == '\\');
        if (!atSeparator && *c != '\0') continue;

        const char saved = *c;
        *c = '\0';
        if (buffer[0] != '\0' && _mkdir(buffer) != 0 && errno != EEXIST) {
            fprintf(stderr, "CAPTURE: could not create directory '%s': %s\n", buffer,
                    strerror(errno));
        }
        *c = saved;
        if (saved == '\0') break;
    }
}

/* Transient adapter so the accumulator does not need to know about Game. It is a stack
 * value passed as an argument and is never stored in persistent state, so the
 * "no function pointers in Game" rule is untouched. */
static void platform_fixed_update(void *ctx, float dt)
{
    game_fixed_update((Game *)ctx, dt);
}
typedef struct {
    bool smokeTest;
    const char *captureScene;
    const char *outputPath;
    int width;
    int height;
    int ticks; /* -1: use the scene's own tick count */
    uint32_t seed;
    int galleryPage;     /* 0: not a gallery capture; >=1: the 1-based page */
    bool video;          /* --capture-scene writes an MP4 of every frame, not one PNG */
    const char *track;   /* NULL: leave whatever game_init() loaded */
    float cameraZoom;    /* 0: leave the default; >0 overrides, for whole-track framing */
    bool validateLap;    /* --validate-lap subcommand */
    const char *carId;   /* --car ID */
    int startCheckpoint; /* ordered checkpoint used as the standing-start pose */
    bool noVideo;        /* --no-video flag */
    bool listCars;       /* --list-cars flag */
} Options;

/*
 * Apply the run options that only the game module can act on. Track and vehicle code lives in
 * the reloadable module, so the platform layer hands over plain data and lets
 * game_configure_run() do the work. Unknown names are reported rather than silently ignored.
 */
static bool apply_run_options(Game *game, const Options *options)
{
    GameRunConfig config;
    memset(&config, 0, sizeof(config));
    config.cameraZoomOverride = options->cameraZoom;
    config.track = GAME_TRACK_KEEP;

    if (options->track != NULL) {
        if (strcmp(options->track, "chicane") == 0) {
            config.track = GAME_TRACK_CHICANE;
        } else if (strcmp(options->track, "sprint") == 0) {
            config.track = GAME_TRACK_SPRINT;
        } else if (strcmp(options->track, "technical") == 0) {
            config.track = GAME_TRACK_TECHNICAL;
        } else if (strcmp(options->track, "lot") == 0 ||
                   strcmp(options->track, "parking_lot") == 0) {
            config.track = GAME_TRACK_PARKING_LOT;
        } else {
            fprintf(
                stderr,
                "error: unknown track '%s' (try 'chicane', 'sprint', 'technical', or 'lot')\n",
                options->track);
            return false;
        }
    }
    return game_configure_run(game, &config);
}

static void print_usage(const char *argv0)
{
    printf("usage: %s [--smoke-test]\n", argv0);
    printf("       %s --capture-scene NAME [--width W] [--height H] [--ticks N]\n", argv0);
    printf("               [--seed N] [--output PATH] [--video]\n");
    printf("       %s --validate-lap [--car ID] [--track NAME] [--start-checkpoint N] "
           "[--output DIR] [--no-video]\n",
           argv0);
    printf("       %s --list-cars\n", argv0);
    printf("       %s --list-scenes\n", argv0);
}

/* Returns false when the caller should exit immediately with *statusOut. */
static bool parse_options(int argc, char **argv, Options *options, int *statusOut)
{
    options->smokeTest = false;
    options->captureScene = NULL;
    options->outputPath = NULL;
    options->width = SCREEN_W;
    options->height = SCREEN_H;
    options->ticks = -1;
    options->seed = 0u;
    options->galleryPage = 0;
    options->video = false;
    options->track = NULL;
    options->cameraZoom = 0.0f;
    options->validateLap = false;
    options->carId = NULL;
    options->startCheckpoint = 0;
    options->noVideo = false;
    options->listCars = false;
    *statusOut = 0;

    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        const bool hasValue = (i + 1 < argc);

        if (strcmp(arg, "--smoke-test") == 0) {
            options->smokeTest = true;
        } else if (strcmp(arg, "--list-scenes") == 0) {
            for (int s = 0; s < CAPTURE_SCENE_COUNT; s++) {
                printf("%-16s scenario=%-16s ticks=%d\n", g_captureScenes[s].name,
                       g_captureScenes[s].scenario, g_captureScenes[s].ticks);
            }
            return false;
        } else if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
            print_usage(argv[0]);
            return false;
        } else if (strcmp(arg, "--capture-scene") == 0 && hasValue) {
            options->captureScene = argv[++i];
        } else if (strcmp(arg, "--output") == 0 && hasValue) {
            options->outputPath = argv[++i];
        } else if (strcmp(arg, "--width") == 0 && hasValue) {
            options->width = atoi(argv[++i]);
        } else if (strcmp(arg, "--height") == 0 && hasValue) {
            options->height = atoi(argv[++i]);
        } else if (strcmp(arg, "--ticks") == 0 && hasValue) {
            options->ticks = atoi(argv[++i]);
        } else if (strcmp(arg, "--seed") == 0 && hasValue) {
            options->seed = (uint32_t)strtoul(argv[++i], NULL, 10);
        } else if (strcmp(arg, "--gallery-page") == 0 && hasValue) {
            options->galleryPage = atoi(argv[++i]);
        } else if (strcmp(arg, "--video") == 0) {
            options->video = true;
        } else if (strcmp(arg, "--track") == 0 && hasValue) {
            options->track = argv[++i];
        } else if (strcmp(arg, "--camera-zoom") == 0 && hasValue) {
            options->cameraZoom = (float)atof(argv[++i]);
        } else if (strcmp(arg, "--validate-lap") == 0) {
            options->validateLap = true;
        } else if (strcmp(arg, "--car") == 0 && hasValue) {
            options->carId = argv[++i];
        } else if (strcmp(arg, "--start-checkpoint") == 0 && hasValue) {
            options->startCheckpoint = atoi(argv[++i]);
        } else if (strcmp(arg, "--no-video") == 0) {
            options->noVideo = true;
        } else if (strcmp(arg, "--list-cars") == 0) {
            options->listCars = true;
        } else {
            fprintf(stderr, "error: unrecognised argument '%s'\n", arg);
            print_usage(argv[0]);
            *statusOut = 2;
            return false;
        }
    }

    if (options->width < 320 || options->height < 240) {
        fprintf(stderr, "error: --width/--height must be at least 320x240\n");
        *statusOut = 2;
        return false;
    }
    return true;
}
/*
 * Deterministic capture. The simulation is advanced with an exact fixed dt rather than
 * GetFrameTime(), so the image depends only on the scene, the tick count, and the build —
 * never on how fast the machine happened to be.
 */
static int run_capture(Game *game, const Options *options)
{
    const CaptureScene *scene = find_capture_scene(options->captureScene);
    if (scene == NULL) {
        fprintf(stderr, "error: no capture scene named '%s' (try --list-scenes)\n",
                options->captureScene);
        return 2;
    }

    const int scenarioIndex = dev_scenario_find(scene->scenario);
    const int ticks = (options->ticks >= 0) ? options->ticks : scene->ticks;
    const char *output = (options->outputPath != NULL) ? options->outputPath
                                                       : "artifacts/screenshots/capture.png";

    game_init(game);
    if (!apply_run_options(game, options)) return 2;
    game->debugOverlay = scene->debugOverlay;
    game->dev.labVisible = scene->lab;
    game->dev.uiDeterministic = true;
    game->dev.scopePreset = scene->scopePreset;
    game->dev.showLoads = scene->showLoads;
    game->dev.showResistance = scene->showResistance;

    /* The platform layer drives the scenario by writing plain data into the Game block; the
     * module's fixed update is what actually applies the script. dev_state_scenario_start()
     * is not reachable from here — it lives inside the reloadable module — and does not need
     * to be: game_init() has already put the simulation in its reset state. */
    if (scenarioIndex > 0) {
        const DevScenario *scenario = dev_scenario_at(scenarioIndex);
        game->dev.scenario = scenarioIndex;
        game->dev.scenarioRunning = true;
        game->dev.scenarioStartTick = game->sim.tick;
        game->dev.seed = (options->seed != 0u) ? options->seed : scenario->seed;
    } else if (options->seed != 0u) {
        game->dev.seed = options->seed;
    }

    for (int i = 0; i < ticks; i++) {
        game_fixed_update(game, FIXED_DT_S);
    }

    /* Two frames: the first lets raylib settle its own per-frame state, the second is the
     * one that gets captured. Both use interpolation alpha 0 so the drawn pose is exactly
     * the last simulated one. */
    game_draw(game, 0.0f);
    game_draw(game, 0.0f);

    ensure_parent_directory(output);
    TakeScreenshot(output);

    printf("CAPTURE: scene=%s ticks=%d size=%dx%d checksum=%08x -> %s\n", scene->name, ticks,
           options->width, options->height, game->stateChecksum, output);
    return 0;
}

/*
 * Deterministic video capture. Same contract as run_capture() above — the simulation is
 * advanced with an exact fixed dt, never GetFrameTime() — except that every frame is read
 * back and streamed to ffmpeg instead of only the last one being written as a PNG.
 *
 * TICKS_PER_FRAME couples the two rates: at FIXED_HZ 120 and 2 ticks per frame the video is
 * 60 fps, and frame N is simulation tick 2N. Telemetry sampled on the same boundary therefore
 * lines up row-for-frame, which is what lets a suspicious video frame be looked up directly
 * in the CSV.
 *
 * Frames go down a pipe rather than to disk: a 60 s run is ~3600 frames and ~13 GB of raw
 * RGBA, which is fine to stream and absurd to store.
 */
#define VIDEO_TICKS_PER_FRAME 2
#define VIDEO_FPS (FIXED_HZ / VIDEO_TICKS_PER_FRAME)

static int run_capture_video(Game *game, const Options *options)
{
    const CaptureScene *scene = find_capture_scene(options->captureScene);
    if (scene == NULL) {
        fprintf(stderr, "error: no capture scene named '%s' (try --list-scenes)\n",
                options->captureScene);
        return 2;
    }

    const int scenarioIndex = dev_scenario_find(scene->scenario);
    const int ticks = (options->ticks >= 0) ? options->ticks : scene->ticks;
    const char *output =
        (options->outputPath != NULL) ? options->outputPath : "artifacts/video/capture.mp4";

    ensure_parent_directory(output);

    /* -an: no audio track. -crf 20: visually lossless enough to read a HUD overlay off.
     * yuv420p rather than the native rgba so the result plays in every consumer player. */
    char command[1024];
    snprintf(command, sizeof(command),
             "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgba -s %dx%d -r %d -i - "
             "-an -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p \"%s\"",
             options->width, options->height, VIDEO_FPS, output);

    /* "wb" is not optional on Windows: text mode would translate every 0x0A byte in the
     * pixel stream into 0x0D 0x0A and corrupt the video. */
    FILE *pipe = _popen(command, "wb");
    if (pipe == NULL) {
        fprintf(stderr, "error: could not start ffmpeg. Is it on PATH?\n");
        return 3;
    }

    game_init(game);

    /* game_init() leaves a windowed build in STATE_MENU, and game_fixed_update() runs no
     * physics outside STATE_PLAYING. A scripted scenario only writes held controls, never the
     * pause press that would start the game, so without this the simulation sits still for
     * the whole capture and the video shows a stationary car under the menu overlay. The
     * headless build already does exactly this in game_init(). */
    game->state = STATE_PLAYING;

    if (!apply_run_options(game, options)) {
        _pclose(pipe);
        return 2;
    }

    game->debugOverlay = scene->debugOverlay;
    game->dev.labVisible = scene->lab;
    game->dev.uiDeterministic = true;
    game->dev.scopePreset = scene->scopePreset;
    game->dev.showLoads = scene->showLoads;
    game->dev.showResistance = scene->showResistance;

    if (scenarioIndex > 0) {
        const DevScenario *scenario = dev_scenario_at(scenarioIndex);
        game->dev.scenario = scenarioIndex;
        game->dev.scenarioRunning = true;
        game->dev.scenarioStartTick = game->sim.tick;
        game->dev.seed = (options->seed != 0u) ? options->seed : scenario->seed;
    }

    const int frameCount = ticks / VIDEO_TICKS_PER_FRAME;
    const clock_t started = clock();
    double readbackSeconds = 0.0;
    int framesWritten = 0;
    bool writeFailed = false;

    for (int frame = 0; frame < frameCount && !writeFailed; frame++) {
        for (int sub = 0; sub < VIDEO_TICKS_PER_FRAME; sub++) {
            game_fixed_update(game, FIXED_DT_S);
        }

        /* Interpolation alpha 0: the drawn pose is exactly the last simulated one, so a video
         * frame shows a state that also exists in the telemetry rather than a blend of two. */
        game_draw(game, 0.0f);

        const clock_t readStart = clock();
        Image shot = LoadImageFromScreen();
        readbackSeconds += (double)(clock() - readStart) / CLOCKS_PER_SEC;

        if (shot.data == NULL) {
            fprintf(stderr, "error: LoadImageFromScreen returned no data at frame %d\n", frame);
            writeFailed = true;
            break;
        }
        const size_t bytes = (size_t)shot.width * (size_t)shot.height * 4u;
        if (fwrite(shot.data, 1, bytes, pipe) != bytes) {
            fprintf(stderr, "error: short write to ffmpeg at frame %d\n", frame);
            writeFailed = true;
        }
        UnloadImage(shot);
        framesWritten++;
    }

    const int status = _pclose(pipe);
    const double elapsed = (double)(clock() - started) / CLOCKS_PER_SEC;

    if (writeFailed) return 3;
    if (status != 0) {
        fprintf(stderr, "error: ffmpeg exited %d\n", status);
        return 3;
    }

    printf("VIDEO: scene=%s frames=%d (%d ticks) %dx%d @%d fps checksum=%08x -> %s\n",
           scene->name, framesWritten, ticks, options->width, options->height, VIDEO_FPS,
           game->stateChecksum, output);
    printf("VIDEO: %.2f s total, %.2f ms/frame (readback %.2f ms/frame, %.0f%% of it)\n",
           elapsed, elapsed * 1000.0 / (double)framesWritten,
           readbackSeconds * 1000.0 / (double)framesWritten, 100.0 * readbackSeconds / elapsed);
    return 0;
}

/*
 * Bounded gallery capture: no simulation, one page of the vehicle corpus drawn through the
 * production texture path, one screenshot, exit. Selected through the DevState field the
 * module already owns, so this needs no new entry point.
 */
static int run_gallery(Game *game, const Options *options)
{
    const char *output = (options->outputPath != NULL) ? options->outputPath
                                                       : "artifacts/gallery-ingame/page.png";

    game_init(game);
    game->dev.uiDeterministic = true;
    game->dev.galleryPage = options->galleryPage;

    /* Two frames for the same reason a scene capture takes two: the first lets raylib settle
     * its own per-frame state, the second is the one that gets written. */
    game_draw(game, 0.0f);
    game_draw(game, 0.0f);

    ensure_parent_directory(output);
    TakeScreenshot(output);

    printf("GALLERY: page=%d size=%dx%d -> %s\n", options->galleryPage, options->width,
           options->height, output);
    return 0;
}

static int run_validate_lap(Game *game, const Options *options)
{
    const char *carId = (options->carId != NULL) ? options->carId : "rwd_grip";
    const int carIndex = car_roster_find(carId);

    const char *outDir =
        (options->outputPath != NULL) ? options->outputPath : "artifacts/validation/latest";
    char carDir[512];
    snprintf(carDir, sizeof(carDir), "%s/%s", outDir, carId);
    telemetry_ensure_dir(carDir);

    char jsonPath[512], csvPath[512], videoPath[512], replayPath[512];
    snprintf(jsonPath, sizeof(jsonPath), "%s/run.json", carDir);
    snprintf(csvPath, sizeof(csvPath), "%s/telemetry.csv", carDir);
    snprintf(videoPath, sizeof(videoPath), "%s/run.mp4", carDir);
    snprintf(replayPath, sizeof(replayPath), "%s/replay.txt", carDir);

    if (carIndex < 0) {
        fprintf(stderr, "error: unknown car id '%s'\n", carId);
        RunReportInput rep;
        memset(&rep, 0, sizeof(rep));
        rep.runId = "invalid_car";
        rep.carId = carId;
        rep.status = RUN_FAIL_SPEC_INVALID;
        run_report_write(jsonPath, &rep);
        return 2;
    }

    VehicleSpec spec;
    if (!car_roster_spec(carIndex, &spec)) {
        fprintf(stderr, "error: failed to build spec for car '%s'\n", carId);
        RunReportInput rep;
        memset(&rep, 0, sizeof(rep));
        rep.runId = "invalid_spec";
        rep.carId = carId;
        rep.status = RUN_FAIL_SPEC_INVALID;
        run_report_write(jsonPath, &rep);
        return 2;
    }

    char specHashHex[16];
    snprintf(specHashHex, sizeof(specHashHex), "%08x", car_roster_spec_hash(carIndex));

    FILE *pipe = NULL;
    if (!options->noVideo) {
        char command[1024];
        snprintf(command, sizeof(command),
                 "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgba -s %dx%d -r %d -i - "
                 "-an -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p \"%s\"",
                 options->width, options->height, VIDEO_FPS, videoPath);
        pipe = _popen(command, "wb");
        if (pipe == NULL) {
            fprintf(stderr, "error: could not start ffmpeg for video encoding\n");
            RunReportInput rep;
            memset(&rep, 0, sizeof(rep));
            rep.runId = "ffmpeg_failed";
            rep.carId = carId;
            rep.status = RUN_FAIL_VIDEO_ENCODE_FAILED;
            run_report_write(jsonPath, &rep);
            return 3;
        }
    }
    game_init(game);
    TelemetryWriter csvWriter;
    const bool csvOpened = telemetry_open(&csvWriter, csvPath);
    game_apply_spec(game, &spec);

    GameRunConfig runConfig;
    memset(&runConfig, 0, sizeof(runConfig));
    runConfig.track = GAME_TRACK_CHICANE;
    if (options->track != NULL) {
        if (strcmp(options->track, "sprint") == 0) {
            runConfig.track = GAME_TRACK_SPRINT;
        } else if (strcmp(options->track, "technical") == 0) {
            runConfig.track = GAME_TRACK_TECHNICAL;
        } else if (strcmp(options->track, "chicane") != 0) {
            fprintf(stderr, "error: unknown validation track '%s'\n", options->track);
            return 2;
        }
    }
    if (!game_configure_run(game, &runConfig) ||
        !game_spawn_on_track_at(game, options->startCheckpoint)) {
        fprintf(stderr, "error: invalid validation start checkpoint %d\n",
                options->startCheckpoint);
        return 2;
    }

    game->autoTrans.enabled = true;
    game->autoTrans.forwardOnly = true;
    game->state = STATE_PLAYING;
    game->dev.uiDeterministic = true;

    AiDriverConfig aiCfg;
    ai_driver_config_default(&aiCfg);
    AiDriverState aiState;
    memset(&aiState, 0, sizeof(aiState));

    const int budgetTicks = 14400; /* 120 s max */
    const int maxRows = budgetTicks / VIDEO_TICKS_PER_FRAME;
    TelemetryRow *rowsBuffer = (TelemetryRow *)calloc((size_t)maxRows, sizeof(TelemetryRow));

    int rowCount = 0;
    int ticksRun = 0;
    int outOfOrder = 0;
    bool allFinite = true;
    bool writeFailed = false;

    replay_begin_recording(&game->replay, game->sim.tick);

    for (int t = 0; t < budgetTicks && game->track.lap < 2 && !writeFailed; t++) {
        ai_driver_update(&aiCfg, &aiState, &game->track, &game->vehicle, &game->derived,
                         &game->spec, &game->input, FIXED_DT_S);
        game_fixed_update(game, FIXED_DT_S);
        ticksRun++;

        if (game->lastCheckpointEvent.outOfOrder) outOfOrder++;
        if (!isfinite(game->vehicle.positionM.x) || !isfinite(game->vehicle.positionM.y))
            allFinite = false;

        if (ticksRun % VIDEO_TICKS_PER_FRAME == 0) {
            TelemetryRow row = game_telemetry_row(game, 1);
            if (rowCount < maxRows && rowsBuffer != NULL) {
                rowsBuffer[rowCount++] = row;
            }
            if (csvOpened) {
                telemetry_write_row(&csvWriter, &row);
            }

            if (pipe != NULL) {
                ValidationOverlayData vod;
                memset(&vod, 0, sizeof(vod));
                vod.carId = carId;
                vod.carDisplayName = car_roster_display_name(carIndex);
                vod.elapsedS = (float)game->sim.tick * FIXED_DT_S;
                vod.checkpointIndex = game->track.nextCheckpoint;
                vod.checkpointTotal = game->track.checkpointCount;
                vod.speedMps = game->derived.speedMps;
                vod.lapState = (game->track.lap < 1) ? 0 : 1;
                vod.isPassing = (outOfOrder == 0 && allFinite);
                vod.steerInput = game->input.steer;
                vod.throttleInput = game->input.throttle;
                vod.brakeInput = game->input.brake;

                render_set_validation_overlay(&vod);
                game_draw(game, 0.0f);

                Image shot = LoadImageFromScreen();
                if (shot.data == NULL) {
                    writeFailed = true;
                } else {
                    const size_t bytes = (size_t)shot.width * (size_t)shot.height * 4u;
                    if (fwrite(shot.data, 1, bytes, pipe) != bytes) writeFailed = true;
                    UnloadImage(shot);
                }
            }
        }
    }

    if (game->lastCheckpointEvent.crossed) {
        TelemetryRow finalRow = game_telemetry_row(game, 1);
        if (rowCount < maxRows && rowsBuffer != NULL) {
            rowsBuffer[rowCount++] = finalRow;
        }
        if (csvOpened) {
            telemetry_write_row(&csvWriter, &finalRow);
        }
        if (pipe != NULL) {
            ValidationOverlayData vod;
            memset(&vod, 0, sizeof(vod));
            vod.carId = carId;
            vod.carDisplayName = car_roster_display_name(carIndex);
            vod.elapsedS = (float)game->sim.tick * FIXED_DT_S;
            vod.checkpointIndex = game->track.nextCheckpoint;
            vod.checkpointTotal = game->track.checkpointCount;
            vod.speedMps = game->derived.speedMps;
            vod.lapState = (game->track.lap < 1) ? 0 : 1;
            vod.isPassing = (outOfOrder == 0 && allFinite);
            vod.steerInput = game->input.steer;
            vod.throttleInput = game->input.throttle;
            vod.brakeInput = game->input.brake;

            render_set_validation_overlay(&vod);
            game_draw(game, 0.0f);

            Image shot = LoadImageFromScreen();
            if (shot.data != NULL) {
                const size_t bytes = (size_t)shot.width * (size_t)shot.height * 4u;
                (void)fwrite(shot.data, 1, bytes, pipe);
                UnloadImage(shot);
            }
        }
    }

    if (csvOpened) telemetry_close(&csvWriter);

    int ffmpegStatus = 0;
    if (pipe != NULL) {
        ffmpegStatus = _pclose(pipe);
    }

    ValidationMetrics metrics;
    if (rowsBuffer != NULL) {
        validation_metrics_compute(rowsBuffer, rowCount, &metrics);
    } else {
        memset(&metrics, 0, sizeof(metrics));
    }

    RunStatus status = RUN_PASS;
    if (!allFinite) {
        status = RUN_FAIL_INVALID_STATE;
    } else if (writeFailed || (pipe != NULL && ffmpegStatus != 0)) {
        status = RUN_FAIL_VIDEO_ENCODE_FAILED;
    } else if (outOfOrder > 0) {
        status = RUN_FAIL_CHECKPOINT_OUT_OF_ORDER;
    } else if (game->track.lap < 2) {
        status = RUN_FAIL_CHECKPOINT_MISSED;
    } else if (ticksRun >= budgetTicks) {
        status = RUN_FAIL_TICK_BUDGET_EXCEEDED;
    }

    char checksumHex[16];
    snprintf(checksumHex, sizeof(checksumHex), "%08x", game->stateChecksum);

    char geometryHashHex[16];
    snprintf(geometryHashHex, sizeof(geometryHashHex), "%08x",
             track_geometry_hash(&game->track));

    bool replaySaved =
        dev_replay_save(&game->replay, replayPath, "validate-lap", 0u, game->stateChecksum);

    char runId[128];
    snprintf(runId, sizeof(runId), "%s-%s-start%d", game->track.id, carId,
             options->startCheckpoint);

    RunReportInput rep;
    memset(&rep, 0, sizeof(rep));
    rep.runId = runId;
    rep.carId = carId;
    rep.carDisplayName = car_roster_display_name(carIndex);
    rep.carDrivetrain = car_roster_layout_name(carIndex);
    rep.carMassKg = (double)game->spec.massKg;
    rep.carSpecHash = specHashHex;
    rep.trackId = game->track.id;
    rep.trackVersion = game->track.version;
    rep.trackGeometryHash = geometryHashHex;
    rep.trackCheckpointCount = game->track.checkpointCount;
    rep.trackLengthM = (double)track_length_m(&game->track);
    rep.startCheckpointIndex = options->startCheckpoint;
    rep.fixedHz = FIXED_HZ;
    rep.telemetryHz = VIDEO_FPS;
    rep.videoFps = VIDEO_FPS;
    rep.buildCommit = DRIFTY_BUILD_COMMIT;
    rep.buildDirty =
        (strcmp(DRIFTY_BUILD_DIRTY, "clean") != 0 && strcmp(DRIFTY_BUILD_DIRTY, "false") != 0);
    rep.finalStateChecksum = checksumHex;
    rep.tickBudget = budgetTicks;
    rep.ticksRun = ticksRun;
    rep.status = status;
    rep.checkpointsPassed = metrics.checkpointsPassed;
    rep.checkpointsMissed =
        (status == RUN_PASS) ? 0 : (game->track.checkpointCount - metrics.checkpointsPassed);
    rep.outOfOrderEvents = outOfOrder;
    rep.metrics = &metrics;
    rep.hasVideo = (!options->noVideo && pipe != NULL && !writeFailed);
    rep.hasReplay = replaySaved;

    run_report_write(jsonPath, &rep);

    printf("VALIDATE: car=%s status=%s timed_lap=%.3fs out_lap=%.3fs -> %s\n", carId,
           run_status_label(status), metrics.timedLapTimeS, metrics.outLapTimeS, jsonPath);

    free(rowsBuffer);
    return (status == RUN_PASS) ? 0 : 1;
}

int main(int argc, char **argv)
{
    Options options;
    int parseStatus = 0;
    if (!parse_options(argc, argv, &options, &parseStatus)) return parseStatus;

    // cppcheck-suppress knownConditionTrueFalse
    if (!Game_LoadModule()) {
        fprintf(stderr, "FATAL: could not load the game module '%s'. Run build.bat first.\n",
                GAME_MODULE_NAME);
        return 1;
    }

    if (options.listCars) {
        const int count = car_roster_count();
        for (int i = 0; i < count; i++) {
            char id[64];
            car_roster_id(i, id, sizeof(id));
            printf("%s\n", id);
        }
        Game_UnloadModule();
        return 0;
    }

    Game *game = (Game *)calloc(1, sizeof(Game));
    if (game == NULL) {
        fprintf(stderr, "FATAL: could not allocate the Game block (%zu bytes)\n", sizeof(Game));
        Game_UnloadModule();
        return 1;
    }

    if (options.validateLap) {
        if (!options.noVideo) {
            InitWindow(options.width, options.height, "Drifty - lap validation");
            if (!IsWindowReady()) {
                fprintf(stderr, "FATAL: InitWindow failed (%dx%d)\n", options.width,
                        options.height);
                Game_UnloadModule();
                free(game);
                return 1;
            }
            SetTargetFPS(0);
        }
        const int status = run_validate_lap(game, &options);
        if (!options.noVideo) {
            CloseWindow();
        }
        game_shutdown(game);
        Game_UnloadModule();
        free(game);
        return status;
    }

    const bool smoke_test = options.smokeTest;
    const bool capture = (options.captureScene != NULL);
    const bool gallery = (options.galleryPage > 0);

    /*
     * Bounded capture modes are offline renders, not playback: the frame limiter would pace
     * them to real time for no benefit, so a 60 s lap would take 60 s to encode and the
     * measured cost would be the limiter rather than the work. Only the interactive and smoke
     * paths, which a human actually watches, are paced. Determinism is unaffected either way —
     * these modes step the simulation with an exact fixed dt and never read GetFrameTime().
     */
    SetTargetFPS((capture || gallery) ? 0 : TARGET_FPS);

    if (capture || gallery) {
        const int status = capture ? (options.video ? run_capture_video(game, &options)
                                                    : run_capture(game, &options))
                                   : run_gallery(game, &options);
        game_shutdown(game);
        CloseWindow();
        Game_UnloadModule();
        free(game);
        return status;
    }

    game_init(game);
    if (!apply_run_options(game, &options)) {
        game_shutdown(game);
        CloseWindow();
        Game_UnloadModule();
        free(game);
        return 2;
    }
    if (smoke_test) {
        /* Force the debug overlay so the screenshot captures reload status and held-input
         * lines without requiring an F1 press. */
        game->debugOverlay = true;
    }

    int smoke_frames = 0;
    int exit_status = 0;

    while (!WindowShouldClose()) {
        /* Development builds only; compiles to nothing in a release build. */
        Game_MaybeHotReload(game);

        /* Sampled once per render frame; held controls stay valid for every substep. */
        input_sample(&game->input);

        /*
         * Physics Lab time control. The lab writes these three fields; the platform loop is
         * the only thing that reads them, so pausing and stepping cannot perturb the
         * simulation itself — a stepped tick is bit-identical to a free-running one.
         *
         * While paused, the accumulator is zeroed before a step so that exactly the
         * requested number of fixed updates run, and stepping more than the substep cap
         * simply spreads across the next frames instead of being counted as backlog.
         */
        float frameTimeS = GetFrameTime() * game->dev.timeScale;
        if (game->dev.paused) {
            if (game->dev.stepTicks > 0) {
                const int stepping = (game->dev.stepTicks < MAX_PHYSICS_STEPS)
                                         ? game->dev.stepTicks
                                         : MAX_PHYSICS_STEPS;
                game->dev.stepTicks -= stepping;
                game->accumulatorS = 0.0f;
                frameTimeS = FIXED_DT_S * (float)stepping;
            } else {
                frameTimeS = 0.0f;
            }
        }

        const TimestepResult step =
            timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops, frameTimeS,
                             platform_fixed_update, game);
        game->lastSubstepCount = step.substeps;

        game_draw(game, step.interpolationAlpha);

        if (smoke_test) {
            smoke_frames++;
            if (smoke_frames >= SMOKE_TEST_FRAMES) {
                ensure_parent_directory(SMOKE_TEST_SCREENSHOT);
                TakeScreenshot(SMOKE_TEST_SCREENSHOT);
                TRACELOG(LOG_INFO,
                         "SMOKE: completed %d frames (substeps=%d backlog=%d alpha=%.3f "
                         "checksum=%08x reloads=%d)",
                         smoke_frames, game->lastSubstepCount, game->physicsBacklogDrops,
                         (double)step.interpolationAlpha, game->stateChecksum,
                         game->reloadCount);
                break;
            }
        }
    }

    if (smoke_test && smoke_frames < SMOKE_TEST_FRAMES) {
        fprintf(stderr, "SMOKE: window closed early after %d / %d frames\n", smoke_frames,
                SMOKE_TEST_FRAMES);
        exit_status = 1;
    }

    game_shutdown(game);

    CloseWindow();
    Game_UnloadModule();
    free(game);

    if (smoke_test && exit_status == 0) {
        printf("SMOKE: ok (%d frames) -> %s\n", smoke_frames, SMOKE_TEST_SCREENSHOT);
    }
    return exit_status;
}
