/*
 * simulation_fixture.c — thin delegate.
 *
 * The Game -> TelemetryRow projection now lives in the game module as game_telemetry_row(),
 * so the headless harness and the --validate-lap pipeline share one mapping and cannot drift.
 * This wrapper keeps the historical test-side name so existing callers are unchanged.
 */
#include "simulation_fixture.h"

TelemetryRow test_telemetry_row_from_game(const Game *game, int substepCount)
{
    return game_telemetry_row(game, substepCount);
}
