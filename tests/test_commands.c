/*
 * test_commands.c — the non-scenario modes: throughput, the failure-bundle verifier, and the
 * parameter/corpus generators.
 *
 * These write artifacts and measure things. Keeping them out of the scenario registry is what
 * lets the registry stay a list of assertions.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#if defined(_WIN32)
#include <direct.h>
#define DRIFTY_RMDIR(path) _rmdir(path)
#else
#include <unistd.h>
#define DRIFTY_RMDIR(path) rmdir(path)
#endif

#include "test_commands.h"
#include "test_scenarios.h"
#include "support/appearance_metrics.h"
#include "support/car_sheet.h"
#include "support/simulation_fixture.h"
#include "support/test_harness.h"

#include "dev/car_corpus.h"
#include "game/car_roster.h"
#include "render/car_visual.h"
#include "render/car_visual_raster.h"
#include "core/config.h"
#include "dev/dev_params.h"
#include "dev/dev_replay.h"
#include "dev/dev_scenario.h"
#include "dev/dev_state.h"
#include "physics/drivetrain.h"
#include "dev/failure_bundle.h"
#include "physics/surface.h"
#include "game/game.h"
#include "game/scoring.h"
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

/* Simulation throughput, for the performance workflow. Prints one machine-readable line. */
int test_run_benchmark(int ticks)
{
    if (ticks <= 0) ticks = 240000;

    Game *game = alloc_game();
    game_init(game);
    game->dev.scenario = dev_scenario_find("power-oversteer");
    game->dev.scenarioRunning = (game->dev.scenario > 0);
    game->dev.scenarioStartTick = 0;

    const clock_t started = clock();
    for (int i = 0; i < ticks; i++) {
        /* Restart the script rather than letting it finish, so every tick does real work. */
        if (!game->dev.scenarioRunning && game->dev.scenario > 0) {
            game->dev.scenarioRunning = true;
            game->dev.scenarioStartTick = game->sim.tick;
        }
        game_fixed_update(game, FIXED_DT_S);
    }
    const clock_t finished = clock();

    const double seconds = (double)(finished - started) / (double)CLOCKS_PER_SEC;
    const double ticksPerSecond = (seconds > 0.0) ? (double)ticks / seconds : 0.0;
    const double realtimeFactor = ticksPerSecond / (double)FIXED_HZ;

    printf("BENCHMARK ticks=%d seconds=%.4f ticks_per_second=%.0f realtime_factor=%.1f "
           "checksum=%08x\n",
           ticks, seconds, ticksPerSecond, realtimeFactor, game->stateChecksum);

    free(game);
    return 0;
}

static bool file_contains_text(const char *path, const char *needle)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) return false;
    char buffer[4096];
    const size_t read = fread(buffer, 1, sizeof(buffer) - 1u, file);
    fclose(file);
    buffer[read] = '\0';
    return strstr(buffer, needle) != NULL;
}

/*
 * Exercise the real bundle writer with a controlled Phase 3 invariant failure. This is an
 * explicit validation mode rather than a normal scenario: the ordinary suite must stay
 * green, while release validation still gets a reproducible way to prove that all six
 * diagnostic files and the Phase 3 context are present. The fixture cleans up after itself.
 */
int test_verify_failure_bundle(const char *rootDir)
{
    const char *root = (rootDir != NULL) ? rootDir : "artifacts/bundle-verification";
    const char *telemetryPath = TELEMETRY_DIR "/_bundle_verification.csv";
    const char *failure =
        "controlled Phase 3 invariant failure: dynamic front load below minimum";
    const char *profile = "Phase3 Candidate";

    Game *game = alloc_game();
    game_init(game);
    const int scenario = dev_scenario_find("accel-load");
    game->dev.scenario = scenario;
    game->dev.scenarioRunning = true;
    game->dev.scenarioStartTick = game->sim.tick;
    replay_begin_recording(&game->replay, game->sim.tick);

    /* --verify-failure-bundle can be the FIRST thing run against a clean checkout. */
    (void)telemetry_ensure_dir(TELEMETRY_DIR);

    TelemetryWriter writer;
    if (!telemetry_open(&writer, telemetryPath)) {
        free(game);
        return 1;
    }
    for (int i = 0; i < 180; i++) {
        game_fixed_update(game, FIXED_DT_S);
        const TelemetryRow row = test_telemetry_row_from_game(game, 1);
        if (!telemetry_write_row(&writer, &row)) {
            telemetry_close(&writer);
            free(game);
            return 1;
        }
    }
    if (!telemetry_close(&writer)) {
        free(game);
        return 1;
    }

    FailureBundle bundle;
    memset(&bundle, 0, sizeof(bundle));
    bundle.scenario = "phase3-controlled";
    bundle.failureText = failure;
    bundle.telemetryPath = telemetryPath;
    bundle.replay = &game->replay;
    bundle.spec = &game->spec;
    bundle.activeProfile = profile;
    bundle.failingTick = game->sim.tick;
    bundle.checksum = game->stateChecksum;
    bundle.seed = 1010u;
    bundle.checksRun = 1;
    bundle.checksFailed = 1;

    char directory[512];
    bool ok = failure_bundle_write(root, &bundle, directory, sizeof(directory));
    static const char *const required[] = { "replay.bin",   "telemetry.csv",
                                            "summary.json", "config_snapshot.txt",
                                            "git_info.txt", "failure.txt" };
    char path[640];
    for (size_t i = 0; ok && i < sizeof(required) / sizeof(required[0]); i++) {
        snprintf(path, sizeof(path), "%s/%s", directory, required[i]);
        FILE *file = fopen(path, "rb");
        ok = (file != NULL);
        if (file != NULL) fclose(file);
    }
    snprintf(path, sizeof(path), "%s/summary.json", directory);
    ok = ok && file_contains_text(path, failure) && file_contains_text(path, profile);
    snprintf(path, sizeof(path), "%s/config_snapshot.txt", directory);
    ok = ok && file_contains_text(path, "active_profile=Phase3 Candidate");

    for (size_t i = 0; i < sizeof(required) / sizeof(required[0]); i++) {
        snprintf(path, sizeof(path), "%s/%s", directory, required[i]);
        remove(path);
    }
    DRIFTY_RMDIR(directory);
    remove(telemetryPath);
    DRIFTY_RMDIR(root);
    free(game);

    printf("FAILURE_BUNDLE_VERIFY files=6 failure_text=yes active_profile=yes cleanup=yes "
           "result=%s\n",
           ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

int test_dump_params(const char *path)
{
    if (path == NULL) {
        dev_params_write_markdown(stdout);
        return 0;
    }

    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        fprintf(stderr, "error: could not write '%s'\n", path);
        return 1;
    }
    dev_params_write_markdown(file);
    if (fclose(file) != 0) {
        fprintf(stderr, "error: could not close '%s'\n", path);
        return 1;
    }
    printf("wrote %s (%d parameters)\n", path, dev_params_count());
    return 0;
}

/* Export every corpus vehicle as a tuning profile, grouped into subdirectories. The files are
 * a human-readable mirror of car_corpus.c, not a second source of truth: the `corpus` scenario
 * asserts each one round-trips back to the spec the code generates. */
int test_generate_corpus(const char *dir)
{
    if (dir == NULL) dir = "data/vehicles/corpus";
    if (!telemetry_ensure_dir(dir)) return 1;

    int written = 0;
    for (int i = 0; i < car_corpus_count(); i++) {
        VehicleSpec spec;
        char id[128], sub[512], path[768];

        if (!car_corpus_spec(i, &spec)) {
            fprintf(stderr, "error: corpus entry %d could not be built\n", i);
            return 1;
        }
        car_corpus_id(i, id, sizeof(id));

        snprintf(sub, sizeof(sub), "%s/%s", dir, car_corpus_group_name(car_corpus_group(i)));
        if (!telemetry_ensure_dir(sub)) return 1;

        snprintf(path, sizeof(path), "%s/%s.txt", sub, id);
        if (!dev_params_save(&spec, path)) {
            fprintf(stderr, "error: could not write '%s'\n", path);
            return 1;
        }
        written++;
    }

    printf("wrote %d vehicle profiles to %s\n", written, dir);
    return 0;
}

/* Print the roster: one id per line, so the suite can enumerate cars from the binary. */
int test_list_cars(void)
{
    for (int i = 0; i < car_roster_count(); i++) {
        char id[128];
        car_roster_id(i, id, sizeof(id));
        printf("%s\n", id);
    }
    return 0;
}

/* Export every roster car as a tuning profile, exactly as --generate-corpus does for the
 * appearance fleet. The `roster` scenario asserts each one round-trips back to the spec
 * the code generates, so the files cannot silently rot away from the code. */
int test_generate_roster(const char *dir)
{
    if (dir == NULL) dir = "data/vehicles/roster";
    if (!telemetry_ensure_dir(dir)) return 1;

    int written = 0;
    for (int i = 0; i < car_roster_count(); i++) {
        VehicleSpec spec;
        char id[128], path[768];

        if (!car_roster_spec(i, &spec)) {
            fprintf(stderr, "error: roster entry %d could not be built\n", i);
            return 1;
        }
        car_roster_id(i, id, sizeof(id));

        snprintf(path, sizeof(path), "%s/%s.txt", dir, id);
        if (!dev_params_save(&spec, path)) {
            fprintf(stderr, "error: could not write '%s'\n", path);
            return 1;
        }
        written++;
    }

    printf("wrote %d roster profiles to %s\n", written, dir);
    return 0;
}

/* The corpus table, in the same generated-Markdown style as --dump-params. */
int test_dump_corpus_index(const char *path)
{
    FILE *out = stdout;
    if (path != NULL) {
        out = fopen(path, "wb");
        if (out == NULL) {
            fprintf(stderr, "error: could not write '%s'\n", path);
            return 1;
        }
    }

    fprintf(out, "<!-- Generated by `build/tests/drifty_tests.exe --dump-corpus-index`."
                 " Do not edit by hand. -->\n");
    fprintf(out, "# Vehicle corpus\n\n");
    fprintf(out, "Every entry is a pure function of its index in `src/dev/car_corpus.c`."
                 " Appearance is derived from these parameters alone —"
                 " see `src/render/car_visual.c`.\n\n");
    fprintf(out, "| # | Group | Id | What defines it |\n|---:|---|---|---|\n");

    for (int i = 0; i < car_corpus_count(); i++) {
        char id[128], note[192];
        car_corpus_id(i, id, sizeof(id));
        car_corpus_describe(i, note, sizeof(note));
        fprintf(out, "| %d | %s | `%s` | %s |\n", i, car_corpus_group_name(car_corpus_group(i)),
                id, note);
    }

    if (path != NULL) {
        if (fclose(out) != 0) {
            fprintf(stderr, "error: could not close '%s'\n", path);
            return 1;
        }
        printf("wrote %s (%d vehicles)\n", path, car_corpus_count());
    }
    return 0;
}

/* ------------------------------------------------------- candidate sweep measurement ----
 *
 * "Measure sweep eligibility, do not assume it" is the standing rule for every new appearance
 * parameter, and the reason is arithmetic: adjacent steps of a five-step row are the tightest
 * pairs in the fleet, so an axis has to clear all three distinctness floors four times over.
 * Two keys that looked certain (tire aspect, body.bed_length) failed, and each cost a full
 * corpus regeneration to find out.
 *
 * This mode answers the question before the axis table is edited. It builds the five cars the
 * generator WOULD build for `key` (car_corpus_sweep_probe, sharing the generator's exclusion
 * window), then reports, per adjacent pair and against the entire existing fleet, the three
 * numbers the corpus scenario asserts. It only measures and prints: nothing is written and no
 * gate is moved. Exit status is 0 when every measured pair clears every floor, 1 otherwise, so
 * it can be used as a decision step rather than read by eye. */
int test_measure_sweep(const char *key)
{
    if (key == NULL) {
        fprintf(stderr, "error: --measure-sweep needs a registry key\n");
        return 2;
    }
    const DevParameter *param = dev_param_find(key);
    if (param == NULL) {
        fprintf(stderr, "error: '%s' is not a registry key\n", key);
        return 2;
    }

    const int steps = car_corpus_sweep_steps();
    const int fleet = car_corpus_count();
    const CarRasterInfo canvas = test_car_shared_canvas(CV_TEST_PX_PER_M);
    const size_t pixels = (size_t)canvas.width * (size_t)canvas.height;
    if (pixels == 0 || steps <= 1) {
        fprintf(stderr, "error: no canvas or no sweep steps\n");
        return 1;
    }

    unsigned char *stepMaps = (unsigned char *)malloc(pixels * (size_t)steps);
    unsigned char *fleetMap = (unsigned char *)malloc(pixels);
    CarVisual *stepVisuals = (CarVisual *)malloc(sizeof(CarVisual) * (size_t)steps);
    VehicleSpec *stepSpecs = (VehicleSpec *)malloc(sizeof(VehicleSpec) * (size_t)steps);
    if (stepMaps == NULL || fleetMap == NULL || stepVisuals == NULL || stepSpecs == NULL) {
        free(stepMaps);
        free(fleetMap);
        free(stepVisuals);
        free(stepSpecs);
        fprintf(stderr, "error: out of memory\n");
        return 1;
    }

    printf("measuring '%s' over [%g, %g] as a %d-step sweep axis at %.1f px/m\n", param->name,
           (double)param->minimum, (double)param->maximum, steps, (double)CV_TEST_PX_PER_M);
    printf("floors: pixels >= %.3f, L2 >= %.2f, Linf >= %.3f m\n\n", (double)CV_MIN_PIXEL_DIFF,
           (double)CV_MIN_L2, (double)CV_MIN_LINF);

    bool built = true;
    for (int s = 0; s < steps && built; s++) {
        if (!car_corpus_sweep_probe(key, s, &stepSpecs[s])) {
            built = false;
            break;
        }
        if (!vehicle_spec_is_valid(&stepSpecs[s])) {
            printf("step %d: value %g is not a valid spec — the axis cannot be swept here\n", s,
                   (double)dev_param_get(&stepSpecs[s], param));
            built = false;
            break;
        }
        car_visual_derive(&stepSpecs[s], &stepVisuals[s]);
        built = car_raster_draw_labels(&stepVisuals[s], canvas, stepMaps + pixels * (size_t)s,
                                       pixels);
    }
    if (!built) {
        free(stepMaps);
        free(fleetMap);
        free(stepVisuals);
        free(stepSpecs);
        fprintf(stderr, "error: could not build the candidate sweep row for '%s'\n", key);
        return 1;
    }

    printf("%-6s %-12s %-9s %-8s %-9s %s\n", "pair", "value", "pixels", "L2", "Linf(m)",
           "worst component");
    bool pass = true;

    /* Adjacent steps first: the tightest pairs, and the ones that kill most candidates. */
    for (int s = 1; s < steps; s++) {
        int worst = 0;
        const float d =
            car_raster_difference(stepMaps + pixels * (size_t)(s - 1),
                                  stepMaps + pixels * (size_t)s, canvas.width, canvas.height);
        const float linf =
            test_car_signature_linf(&stepVisuals[s - 1], &stepVisuals[s], &worst);
        const float l2 = test_car_signature_l2(&stepVisuals[s - 1], &stepVisuals[s]);
        const bool ok = (d >= CV_MIN_PIXEL_DIFF && l2 >= CV_MIN_L2 && linf >= CV_MIN_LINF);
        pass = pass && ok;
        printf("%d-%d    %-12.4g %-9.4f %-8.4f %-9.4f %s%s\n", s - 1, s,
               (double)dev_param_get(&stepSpecs[s], param), (double)d, (double)l2, (double)linf,
               car_visual_signature_component_name(worst), ok ? "" : "   FAIL");
    }

    /* Then the whole existing fleet: a new row has to be distinct from all 100 cars, not only
     * from its own neighbours.
     *
     * Each floor is minimised INDEPENDENTLY over every step x fleet pair. Picking one "closest"
     * vehicle by pixel difference and then reading its L2 and Linf would report a pass whenever
     * some other fleet vehicle differs in more pixels but shares a feature vector — exactly the
     * failure mode the corpus scenario catches, reported here as a clean bill of health. */
    printf("\n%-6s %-9s %-30s %-8s %-30s %-9s %s\n", "step", "pixels", "worst pixel pair", "L2",
           "worst L2 pair", "Linf(m)", "worst Linf pair");
    for (int s = 0; s < steps; s++) {
        float minDiff = 2.0f, minL2 = 1e9f, minLinf = 1e9f;
        int diffIndex = -1, l2Index = -1, linfIndex = -1;
        for (int i = 0; i < fleet; i++) {
            VehicleSpec other;
            if (!car_corpus_spec(i, &other)) continue;
            /* A key that is ALREADY a sweep axis would otherwise measure itself: the fleet
             * contains the very car this step rebuilds, the comparison is 0.0000 on every
             * metric, and the verdict would read as a catastrophic failure instead of a
             * tautology. The comparison walks the registry rather than the struct bytes —
             * VehicleSpec padding is unspecified, so memcmp could call two identical cars
             * different and quietly reinstate the tautology it is here to remove. */
            if (test_car_primary_diff_count(&other, &stepSpecs[s], NULL) == 0) continue;
            if (!test_car_labels_for_spec(&other, canvas, fleetMap, pixels)) continue;

            CarVisual otherVisual;
            int worst = 0;
            car_visual_derive(&other, &otherVisual);

            const float d = car_raster_difference(stepMaps + pixels * (size_t)s, fleetMap,
                                                  canvas.width, canvas.height);
            const float l2 = test_car_signature_l2(&stepVisuals[s], &otherVisual);
            const float linf = test_car_signature_linf(&stepVisuals[s], &otherVisual, &worst);

            if (d < minDiff) {
                minDiff = d;
                diffIndex = i;
            }
            if (l2 < minL2) {
                minL2 = l2;
                l2Index = i;
            }
            if (linf < minLinf) {
                minLinf = linf;
                linfIndex = i;
            }
        }
        char diffId[128] = "?", l2Id[128] = "?", linfId[128] = "?";
        if (diffIndex >= 0) car_corpus_id(diffIndex, diffId, sizeof(diffId));
        if (l2Index >= 0) car_corpus_id(l2Index, l2Id, sizeof(l2Id));
        if (linfIndex >= 0) car_corpus_id(linfIndex, linfId, sizeof(linfId));

        const bool ok =
            (minDiff >= CV_MIN_PIXEL_DIFF && minL2 >= CV_MIN_L2 && minLinf >= CV_MIN_LINF);
        pass = pass && ok;
        printf("%-6d %-9.4f %-30s %-8.4f %-30s %-9.4f %s%s\n", s, (double)minDiff, diffId,
               (double)minL2, l2Id, (double)minLinf, linfId, ok ? "" : "   FAIL");
    }

    printf("\nverdict: '%s' %s carry a sweep row against the current fleet\n", param->name,
           pass ? "CAN" : "CANNOT");

    free(stepMaps);
    free(fleetMap);
    free(stepVisuals);
    free(stepSpecs);
    return pass ? 0 : 1;
}

/* Every corpus vehicle's style axes and signature vector as one CSV, for spread analysis.
 *
 * The `corpus` scenario only answers "is any PAIR closer than the threshold". That is a
 * minimum, and it says nothing about how the fleet uses the space: a corpus can pass while
 * every car sits in one corner of it. This dump exists so the distribution can be looked at
 * directly — per-column range, variance, and which components are effectively constant. */
int test_dump_corpus_metrics(const char *path)
{
    FILE *out = stdout;
    if (path != NULL) {
        out = fopen(path, "wb");
        if (out == NULL) {
            fprintf(stderr, "error: could not write '%s'\n", path);
            return 1;
        }
    }

    const int sigCount = car_visual_signature_count();

    fprintf(out, "index,group,id,mass01,size01,low01,grip01,balance01,power01,aero01,"
                 "sport01,strip01");
    for (int c = 0; c < sigCount; c++) {
        fprintf(out, ",%s", car_visual_signature_component_name(c));
    }
    fprintf(out, "\n");

    for (int i = 0; i < car_corpus_count(); i++) {
        VehicleSpec spec;
        if (!car_corpus_spec(i, &spec)) continue;

        CarVisual visual;
        car_visual_derive(&spec, &visual);

        float sig[128];
        const int written =
            car_visual_signature(&visual, sig, (int)(sizeof(sig) / sizeof(sig[0])));

        char id[128];
        car_corpus_id(i, id, sizeof(id));

        const CarLatents *l = &visual.latents;
        fprintf(out, "%d,%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", i,
                car_corpus_group_name(car_corpus_group(i)), id, l->mass01, l->size01, l->low01,
                l->grip01, l->balance01, l->power01, l->aero01, l->sport01, l->strip01);
        for (int c = 0; c < written; c++) fprintf(out, ",%.6f", sig[c]);
        fprintf(out, "\n");
    }

    if (path != NULL) {
        if (fclose(out) != 0) {
            fprintf(stderr, "error: could not close '%s'\n", path);
            return 1;
        }
        printf("wrote %s (%d vehicles, %d signature components)\n", path, car_corpus_count(),
               sigCount);
    }
    return 0;
}

int test_dump_corpus_cards(const char *dir)
{
    return car_sheet_write_cards(dir, 0.0f) ? 0 : 1;
}

int test_dump_corpus_sheet(const char *dir)
{
    return car_sheet_write(dir, 0.0f, 0) ? 0 : 1;
}
