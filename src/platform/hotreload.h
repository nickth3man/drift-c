/*
 * hotreload.h — the one authoritative list of reloadable entry points.
 *
 * GAME_ENTRY_POINTS is the single source of truth. Adding an entry point means editing one
 * line here; the typedefs, the platform-side function-pointer table, and the symbol lookups
 * all expand from it, so they cannot fall out of sync.
 *
 * Three build configurations select different expansions:
 *
 *   DRIFTY_HOT_RELOAD + !DRIFTY_GAME_MODULE   platform layer: function pointers, resolved
 *                                             from the module at load time.
 *   DRIFTY_HOT_RELOAD +  DRIFTY_GAME_MODULE   game module: exported definitions.
 *   !DRIFTY_HOT_RELOAD                        release: plain prototypes, called directly,
 *                                             no module and no indirection.
 *
 * RELOAD-SAFETY INVARIANT. The platform layer owns the Game allocation and hands the same
 * pointer back to each freshly loaded module. For that to be sound:
 *
 *   - No field in Game, or in anything reachable from it, may point into the game module's
 *     code or static data.
 *   - No function pointers may be stored in persistent state. If an indirection is really
 *     needed, rebuild the table in game_post_reload().
 *   - Refer to module-static tables by id (e.g. SurfaceId), never by pointer, and resolve
 *     the id through an accessor at point of use.
 *   - game_pre_reload() releases anything the module owns that raylib tracks; the matching
 *     game_post_reload() re-acquires it.
 *   - Changing the layout of Game invalidates the live memory block. Restart the
 *     executable after a struct-layout change. This is the main limitation of the
 *     technique, and it is expected rather than a bug.
 *
 * Two build-side contracts belong to this layer and are stated nowhere else:
 *
 * **Write to a temporary filename and rename.** The linker creates the output file before
 * filling it, so a running game polling for changes can load a zero-length module. Building
 * to `game_tmp.dll` and renaming makes the swap atomic.
 *
 * **Emit a fresh PDB per build if you debug on Windows.** A debugger attached to the process
 * holds a lock on the current PDB and the next build fails. Numbering them
 * (`game_1.pdb`, `game_2.pdb`, …) avoids this; clean them up on a fresh start.
 */
#ifndef DRIFTY_HOTRELOAD_H
#define DRIFTY_HOTRELOAD_H

/*
 * The GAME itself is Windows-only and main.c says so with its own #error. The guard here is
 * narrower on purpose: it fires only for a build that actually asks for hot reload, so that
 * the headless test executable — which defines neither DRIFTY_HOT_RELOAD nor DRIFTY_GAME_MODULE
 * and links no raylib library — still compiles on Linux. That is what lets CI run the
 * sanitizer, coverage, and fuzzing jobs, none of which have a Windows equivalent.
 */
#if defined(DRIFTY_HOT_RELOAD) && !defined(_WIN32)
#error Hot reload is implemented for Windows only.
#endif

#include <stdbool.h>

/* Incomplete types: the entry-point signatures only need pointers. game.h completes them. */
typedef struct Game Game;
typedef struct GameRunConfig GameRunConfig;

/* This project spells logging TRACELOG(LOG_LEVEL, ...), but raylib's TRACELOG macro lives
 * in its internal utils.h, which is not installed with the library. Provide the same
 * spelling on top of the public TraceLog(). Usable only where raylib.h is in scope, which
 * is every translation unit that logs. */
#ifndef TRACELOG
#define TRACELOG(level, ...) TraceLog((level), __VA_ARGS__)
#endif

/* Exported from the reloadable module; a no-op everywhere else. */
#if defined(_WIN32) && defined(DRIFTY_HOT_RELOAD) && defined(DRIFTY_GAME_MODULE)
#define GAME_API __declspec(dllexport)
#else
#define GAME_API
#endif

/* Path the platform layer watches and loads. Overridable at build time. */
#ifndef GAME_MODULE_NAME
#define GAME_MODULE_NAME "build/dev/game.dll"
#endif

/* ------------------------------------------------------------------------------------- */

#define GAME_ENTRY_POINTS                                                                 \
    ENTRY(game_init, void, Game *)                /* first-time setup */                  \
    ENTRY(game_pre_reload, void, Game *)          /* release module-owned resources */    \
    ENTRY(game_post_reload, void, Game *)         /* re-acquire them */                   \
    ENTRY(game_fixed_update, void, Game *, float) /* one fixed step */                    \
    ENTRY(game_draw, void, Game *, float)         /* render with interpolation alpha */   \
    ENTRY(game_shutdown, void, Game *)                                                    \
    /* select track and car, then place the car on the start line. The platform layer   \
     * cannot do this itself: track and vehicle code live in the reloadable module. */ \
    ENTRY(game_configure_run, bool, Game *, const GameRunConfig *)

/* Function types, needed by every configuration. */
#define ENTRY(name, ret, ...) typedef ret(name##_t)(__VA_ARGS__);
GAME_ENTRY_POINTS
#undef ENTRY

#if defined(DRIFTY_HOT_RELOAD) && !defined(DRIFTY_GAME_MODULE)

/* Platform layer: calls go through a table resolved from the loaded module. */
#define ENTRY(name, ...) extern name##_t *name;
GAME_ENTRY_POINTS
#undef ENTRY

/* Load the module for the first time and populate the table. False means nothing was
 * loaded and no entry point is callable. */
bool Game_LoadModule(void);

/* Poll the watched module's modification time and, if it changed, validate a candidate
 * module and swap it in. Returns true only when a swap actually completed. A failed
 * candidate leaves the running module and its table untouched. */
bool Game_MaybeHotReload(Game *game);

/* Release the active module. Safe to call when nothing is loaded. */
void Game_UnloadModule(void);

#else

/* Release / game-module configuration: direct calls, no indirection, no DLL. */
#define ENTRY(name, ret, ...) GAME_API ret name(__VA_ARGS__);
GAME_ENTRY_POINTS
#undef ENTRY

static inline bool Game_LoadModule(void)
{
    return true;
}
static inline bool Game_MaybeHotReload(Game *game)
{
    (void)game;
    return false;
}
static inline void Game_UnloadModule(void) {}

#endif

#endif /* DRIFTY_HOTRELOAD_H */
