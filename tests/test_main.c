/*
 * test_main.c — the headless scenario runner: argument parsing, ordered group iteration,
 * failure-bundle dispatch, and the summary line the CI gates read.
 *
 * It opens no window, initialises no audio, requires no display, and calls no raylib function.
 *
 * Usage:
 *     drifty_tests                       run every scenario
 *     drifty_tests --scenario replay     run one scenario
 *     drifty_tests --list                list scenario names
 *     drifty_tests -v                    print passing checks too
 *
 * Exit status is the number of failed checks, clamped to 125, so a failure is nonzero.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "test_commands.h"
#include "test_scenarios.h"
#include "support/car_sheet.h"
#include "support/test_harness.h"

#include "dev/failure_bundle.h"
#include "game/game.h"

static void print_usage(const char *argv0)
{
    printf("usage: %s [--scenario NAME] [--filter PATTERN] [--list] [-v] [--no-bundle] "
           "[--artifacts DIR] [--junit [FILE]]\n",
           argv0);
    printf("       %s --dump-params [PATH]     write the parameter table as Markdown\n", argv0);
    printf("       %s --benchmark [TICKS]      fixed-update throughput, no telemetry\n", argv0);
    printf("       %s --verify-failure-bundle [DIR]  create, inspect, and clean a fixture\n",
           argv0);
    printf("       %s --generate-corpus [DIR]   export the vehicle corpus as tuning profiles\n",
           argv0);
    printf("       %s --dump-corpus-index [PATH] write the corpus table as Markdown\n", argv0);
    printf("       %s --dump-corpus-metrics [PATH] write corpus latents+signatures as CSV\n",
           argv0);
    printf("       %s --dump-corpus-cards [DIR]  per-car PNGs, label maps and cards.json\n",
           argv0);
    printf("       %s --dump-corpus-sheet [DIR]  render the vehicle contact sheet (PNG+HTML)\n",
           argv0);
    printf("       %s --measure-sweep KEY       can this key carry a corpus sweep row?\n",
           argv0);
    printf("       %s --list-cars               list validation roster car ids\n", argv0);
    printf("       %s --generate-roster [DIR]   export roster cars as tuning profiles\n",
           argv0);
}

/* Write one XML-safe character; returns the number of bytes written. */
static int xml_escape_char(char c, char *buf, size_t size)
{
    const char *replace;
    int len;
    switch (c) {
        case '&':
            replace = "&amp;";
            len = 5;
            break;
        case '<':
            replace = "&lt;";
            len = 4;
            break;
        case '>':
            replace = "&gt;";
            len = 4;
            break;
        case '"':
            replace = "&quot;";
            len = 6;
            break;
        case '\'':
            replace = "&apos;";
            len = 6;
            break;
        default:
            if ((unsigned char)c < 0x20) return 0;
            if ((int)size > 0) {
                buf[0] = c;
                buf[1] = '\0';
                return 1;
            }
            return 0;
    }
    if ((int)size < len + 1) return 0;
    memcpy(buf, replace, (size_t)len);
    buf[len] = '\0';
    return len;
}

/* Copy text into buf with XML escaping. buf is always null-terminated. */
static void xml_escape(const char *src, char *buf, size_t size)
{
    size_t pos = 0;
    while (*src && pos + 1 < size) {
        pos += (size_t)xml_escape_char(*src, buf + pos, size - pos);
        src++;
    }
    buf[pos] = '\0';
}

int main(int argc, char **argv)
{
    /* The declaration order here IS the run order, and it is the only thing that decides it. */
    const TestScenarioGroup groups[] = {
        test_core_scenarios(),     test_appearance_scenarios(), test_physics_scenarios(),
        test_handling_scenarios(), test_gameplay_scenarios(),
    };
    const size_t groupCount = sizeof(groups) / sizeof(groups[0]);

    const char *only = NULL;
    const char *artifactsDir = "artifacts";
    const char *filterPattern = NULL;
    const char *junitPath = NULL;
    const char *junitNames[128];
    double junitTimes[128];
    int junitFailures[128];
    int junitCount = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--list") == 0) {
            for (size_t g = 0; g < groupCount; g++) {
                for (size_t s = 0; s < groups[g].count; s++) {
                    printf("%-12s %s\n", groups[g].items[s].name,
                           groups[g].items[s].description);
                }
            }
            return 0;
        }
        if (strcmp(argv[i], "--dump-params") == 0) {
            return test_dump_params((i + 1 < argc) ? argv[i + 1] : NULL);
        }
        if (strcmp(argv[i], "--benchmark") == 0) {
            return test_run_benchmark((i + 1 < argc) ? atoi(argv[i + 1]) : 0);
        }
        if (strcmp(argv[i], "--generate-corpus") == 0) {
            return test_generate_corpus((i + 1 < argc && argv[i + 1][0] != '-') ? argv[i + 1]
                                                                                : NULL);
        }
        if (strcmp(argv[i], "--list-cars") == 0) {
            return test_list_cars();
        }
        if (strcmp(argv[i], "--generate-roster") == 0) {
            return test_generate_roster((i + 1 < argc && argv[i + 1][0] != '-') ? argv[i + 1]
                                                                                : NULL);
        }
        if (strcmp(argv[i], "--dump-corpus-index") == 0) {
            return test_dump_corpus_index((i + 1 < argc && argv[i + 1][0] != '-') ? argv[i + 1]
                                                                                  : NULL);
        }
        if (strcmp(argv[i], "--measure-sweep") == 0) {
            return test_measure_sweep((i + 1 < argc) ? argv[i + 1] : NULL);
        }
        if (strcmp(argv[i], "--dump-corpus-cards") == 0) {
            const char *dir = (i + 1 < argc && argv[i + 1][0] != '-')
                                  ? argv[i + 1]
                                  : "artifacts/corpus-cards";
            return test_dump_corpus_cards(dir);
        }
        if (strcmp(argv[i], "--dump-corpus-metrics") == 0) {
            return test_dump_corpus_metrics(
                (i + 1 < argc && argv[i + 1][0] != '-') ? argv[i + 1] : NULL);
        }
        if (strcmp(argv[i], "--dump-corpus-sheet") == 0) {
            const char *dir =
                (i + 1 < argc && argv[i + 1][0] != '-') ? argv[i + 1] : "artifacts/gallery";
            return test_dump_corpus_sheet(dir);
        }
        if (strcmp(argv[i], "--verify-failure-bundle") == 0) {
            return test_verify_failure_bundle(
                (i + 1 < argc && argv[i + 1][0] != '-') ? argv[i + 1] : NULL);
        }
        if (strcmp(argv[i], "--no-bundle") == 0) {
            test_harness_set_bundles_enabled(false);
            continue;
        }
        if (strcmp(argv[i], "--artifacts") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "error: --artifacts needs a directory\n");
                return 2;
            }
            artifactsDir = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            test_harness_set_verbose(true);
            continue;
        }
        if (strcmp(argv[i], "--scenario") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "error: --scenario needs a name\n");
                print_usage(argv[0]);
                return 2;
            }
            only = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (strcmp(argv[i], "--filter") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "error: --filter needs a pattern\n");
                return 2;
            }
            filterPattern = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--junit") == 0) {
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                junitPath = argv[++i];
            } else {
                junitPath = "artifacts/junit.xml";
            }
            continue;
        }
        fprintf(stderr, "error: unrecognised argument '%s'\n", argv[i]);
        print_usage(argv[0]);
        return 2;
    }

    printf("drifty_tests - Phase 0-3 planar vehicle physics and development tooling\n");
    printf("headless: no window, audio, display, or raylib function call\n\n");

    int ran = 0;
    for (size_t g = 0; g < groupCount; g++) {
        for (size_t s = 0; s < groups[g].count; s++) {
            const TestScenario *scenario = &groups[g].items[s];
            if (only != NULL && strcmp(only, scenario->name) != 0) continue;
            if (filterPattern != NULL && strstr(scenario->name, filterPattern) == NULL)
                continue;

            const TestHarnessSnapshot before = test_harness_snapshot();

            printf("[ %s ] %s\n", scenario->name, scenario->description);
            test_harness_clear_scenario();
            const clock_t t0 = clock();
            scenario->run();
            const clock_t t1 = clock();
            const double elapsedS = (double)(t1 - t0) / CLOCKS_PER_SEC;

            const TestHarnessSnapshot after = test_harness_snapshot();
            const int scenarioChecks = after.checks - before.checks;
            const int scenarioFailures = after.failures - before.failures;
            printf("  -> %d checks, %d failed\n", scenarioChecks, scenarioFailures);

            if (scenarioFailures > 0 && test_harness_bundles_enabled()) {
                FailureBundle bundle;
                memset(&bundle, 0, sizeof(bundle));
                bundle.scenario = scenario->name;
                bundle.failureText = after.firstFailureText;
                bundle.telemetryPath =
                    after.bundleHasTelemetry ? after.bundleTelemetryPath : NULL;
                bundle.replay = (after.bundleGame != NULL) ? &after.bundleGame->replay : NULL;
                bundle.spec = (after.bundleGame != NULL) ? &after.bundleGame->spec : NULL;
                bundle.activeProfile = "Default";
                bundle.failingTick =
                    (after.bundleGame != NULL) ? after.bundleGame->sim.tick : 0u;
                bundle.checksum =
                    (after.bundleGame != NULL) ? after.bundleGame->stateChecksum : 0u;
                bundle.seed = after.bundleSeed;
                bundle.checksRun = scenarioChecks;
                bundle.checksFailed = scenarioFailures;

                char directory[512];
                if (failure_bundle_write(artifactsDir, &bundle, directory, sizeof(directory))) {
                    printf("  -> failure bundle: %s\n", directory);
                } else {
                    fprintf(stderr, "  -> could not write a failure bundle under '%s'\n",
                            artifactsDir);
                }
            }
            printf("\n");
            ran++;

            if (junitPath != NULL && junitCount < 128) {
                junitNames[junitCount] = scenario->name;
                junitTimes[junitCount] = elapsedS;
                junitFailures[junitCount] = scenarioFailures;
                junitCount++;
            }
        }
    }

    /* Every pending bundle has been written above, so the retained Game can go now. */
    test_handling_cleanup();

    if (junitPath != NULL) {
        FILE *jf = fopen(junitPath, "w");
        if (jf == NULL) {
            fprintf(stderr, "error: could not write '%s'\n", junitPath);
        } else {
            double totalTime = 0.0;
            int totalFailures = 0;
            for (int i = 0; i < junitCount; i++) {
                totalTime += junitTimes[i];
                if (junitFailures[i] > 0) totalFailures++;
            }
            fprintf(jf, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
            fprintf(jf,
                    "<testsuite name=\"drifty_tests\" tests=\"%d\" failures=\"%d\" "
                    "errors=\"0\" time=\"%.3f\">\n",
                    junitCount, totalFailures, totalTime);
            for (int i = 0; i < junitCount; i++) {
                char escName[256];
                xml_escape(junitNames[i], escName, sizeof(escName));
                fprintf(jf, "  <testcase name=\"%s\" classname=\"Scenarios\" time=\"%.3f\">",
                        escName, junitTimes[i]);
                if (junitFailures[i] > 0) {
                    fprintf(jf, "\n    <failure message=\"%d check(s) failed\"/>\n",
                            junitFailures[i]);
                    fprintf(jf, "  </testcase>\n");
                } else {
                    fprintf(jf, "</testcase>\n");
                }
            }
            fprintf(jf, "</testsuite>\n");
            fclose(jf);
            printf("JUnit report: %s\n", junitPath);
        }
    }

    if (only != NULL && ran == 0) {
        fprintf(stderr, "error: no scenario named '%s' (try --list)\n", only);
        return 2;
    }

    const TestHarnessSnapshot total = test_harness_snapshot();
    printf("=====================================================\n");
    printf("%d scenario(s), %d checks, %d failed\n", ran, total.checks, total.failures);
    printf("%s\n", (total.failures == 0) ? "PASS" : "FAIL");

    if (total.failures == 0) return 0;
    return (total.failures > 125) ? 125 : total.failures;
}
