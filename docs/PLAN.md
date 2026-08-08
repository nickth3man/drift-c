# Drifty → Racing: Milestone 1 (Automated Chicane Lap Validation)

## Context

Drifty is a top-down 2D driving simulator in C11 + raylib 6.0 (Windows / MSYS2 UCRT64, ~29k LOC). Its
vehicle model is already sim-cade: a planar rigid body with four wheels, longitudinal *and* lateral load
transfer, normalized Pacejka-form tire curves under a per-wheel friction ellipse, slip ratio, tire
relaxation length, load sensitivity, LOCKED/OPEN/LSD differentials, Ackermann, aero drag, per-wheel
rolling resistance, and a four-surface friction table.

The drift focus does **not** live in the force model. It lives in `scoring.c`, in the score HUD, in the
camera zoom, and in the smoke spawn — all of which are asserted (by the `scoring-determinism` scenario)
to never feed back into forces. So this is a pivot of the *gameplay layer*, not a physics rewrite. There
is no anti-drift assist to remove, and none will be added: power oversteer, throttle-induced rotation,
and handbrake slides remain available to any car whose grip, power and drivetrain allow them.

The goal of Milestone 1 is narrow and deliberately mechanical: a chicane test track with an ordered
checkpoint system, an AI driver that can only touch the same `Input` struct a human touches, and a
repeatable pipeline that produces, per car, an MP4 + a telemetry CSV + a run JSON with an explicit
pass/fail. That pipeline is the instrument every later handling change is measured with, so it is built
first.

---

## Decisions taken (from the interview)

| Question | Decision |
|---|---|
| Playable car set | New explicit **6-car roster**, 2 per drivetrain layout (RWD/FWD/AWD), C table, pipeline-only. No in-game car select in M1. |
| Video capture | **Deterministic offline render** — exact `FIXED_DT_S` stepping, raw frames piped to ffmpeg. No wall clock, no key injection. |
| AI controls | **Steer + throttle + brake only**, with the existing automatic transmission enabled. No handbrake, no manual shift. |
| FWD/AWD | **Implement physically in M1.** `drivetrainLayout` / `frontTorqueSplit` become real. |
| Track authoring | **C-authored ribbon centerline + explicit ordered checkpoint array**, versioned. |
| Drift scoring | **Remove entirely**, including the `scoringDrift` classifier. Spin detection rebuilt in the metrics layer. |
| Lap structure | **Out-lap + 1 timed lap.** |
| Telemetry rate | **60 Hz** (every 2nd tick). Simulation stays at **120 Hz**. Video at 60 fps ⇒ CSV row *N* ≡ video frame *N*. |
| Orchestrator | **C subcommand `--validate-lap`** does one run end to end; **Python** aggregates the suite and diffs against previous runs. |
| Run termination | Abort only on **hard tick budget** or **stall**. Out-of-order/missed checkpoints and off-track excursions are **recorded, not aborted** (a missed required checkpoint still means FAIL). |

**Not in Milestone 1:** AI opponents, championships, progression, multiplayer, additional tracks, race
modes, in-game car selection, vsync/display-rate work.

---

## Assessment of existing systems

### Verified by inspection

| System | Where | Disposition |
|---|---|---|
| Fixed 120 Hz timestep, substep cap, interpolation alpha | `src/platform/timestep.c`, `main.c` | **Keep as-is** |
| Hot-reload platform/module split, `Game` block ownership | `src/platform/`, `src/game/game.h` | **Keep as-is** |
| `Input` struct: 4 held controls + 6 one-shots, raylib-free | `src/game/input.h` | **Keep as-is** — this is the AI's only interface |
| Deterministic replay buffer (fixed-tick `Input` timeline) | `src/game/replay.c` | **Keep as-is** — records AI input for free |
| Tire curves, friction ellipse, slip ratio, relaxation | `src/physics/tire.c` | **Keep as-is** |
| Load transfer (long. + lat.), aero drag, rolling resistance | `src/physics/physics.c` | **Keep as-is** |
| Surface friction table (asphalt/gravel/grass/snow) | `src/physics/surface.c` | **Keep as-is** |
| Swept capsule collision vs. track barriers | `src/world/collision.c` | **Reuse with modification** (runoff-edge barriers) |
| Automatic transmission | `src/physics/auto_transmission.c` | **Keep as-is** — the AI relies on it |
| CSV telemetry writer + single `Game`→row projection | `src/game/telemetry.c`, `tests/support/simulation_fixture.c` | **Reuse with modification** (append columns) |
| Scripted-scenario harness pattern | `run_scripted_scenario()` in `tests/scenarios/handling_tests.c` | **Reuse** as the template for the validation run loop |
| Deterministic screenshot capture (`--capture-scene`) | `src/platform/main.c` | **Reuse with modification** → per-frame video |
| Failure bundles (replay + spec + telemetry + checksum) | `src/dev/failure_bundle.c` | **Keep as-is** — reuse for failed runs |
| Telemetry comparison / report tooling | `tools/telemetry/*.py`, `tests/baselines/`, `mk regression` | **Keep as-is** |
| Parameter registry (123 tunables, one definition each) | `src/dev/dev_params.c` | **Keep as-is** |
| `dev_presets.c` (10 hand-tuned driving profiles) | `src/dev/dev_presets.c` | **Keep** — seed values for the new roster |
| `car_corpus.c` (100-vehicle appearance fleet) | `src/dev/car_corpus.c` | **Keep as-is** — appearance workstream, not the roster |
| Drift scoring, combo, high-score persistence | `src/game/scoring.c`, `Game.driftScore` etc. | **Remove** |
| `derived.scoringDrift` classifier | `src/physics/physics.c:1010`, `scoring.c` | **Remove** |
| Score HUD card, DRIFT! callout, drift camera zoom | `src/render/render_hud.c`, `render.c` | **Remove** |
| Parking-lot "track" (400×300 m empty rectangle) | `src/world/track.c` | **Refactor substantially** |
| Implicit per-node checkpoint gates | `track_update_checkpoints()` | **Replace** with explicit ordered checkpoints |
| AI driver | — | **Missing — build** |
| Video capture | — | **Missing — build** |
| Run-level JSON metadata / summary metrics | — | **Missing — build** |
| Car roster / enumeration | — | **Missing — build** |

### Confirmed gaps that change the work

1. **Drive torque is hardcoded RWD.** `drivetrain_calculate_torques()` (`src/physics/drivetrain.c:154`)
   writes drive torque only to `WHEEL_REAR_LEFT/RIGHT`. `drivetrainLayout` and `frontTorqueSplit` are
   registered parameters (`dev_params.c:299`) consumed **only by the appearance grammar**. FWD and AWD
   are currently cosmetic. → Phase 2.
2. **No downforce.** `aeroLiftCoefFront/Rear`, `aeroRefArea*` feed appearance only. → deferred to M2;
   noted so no one assumes it works.
3. **Track surface edge and barrier are the same line.** `Track_SurfaceAt()` and
   `collision_resolve_track()` both use `TrackNode.halfWidthM`, so leaving the racing surface *is*
   hitting the wall. Off-track excursions are therefore unmeasurable today. → Phase 1 adds a runoff band.
4. **`track_update_checkpoints()` only tests the *next* gate**, so an out-of-order crossing is invisible
   — it simply doesn't advance. Recording out-of-order events requires testing all gates. → Phase 1.
5. **`track_init()` is `#if !defined(DRIFTY_HEADLESS)`-gated inside `game_init()`.** `track.c` and
   `collision.c` *are* in the headless link closure (`GAME_SRCS`), so this is a wiring change, not a
   build change. Leaving `game_init()` alone and having the validation path load the track explicitly
   keeps all 8 committed CSV baselines untouched.
6. **Steering speed-sensitivity is disabled** (`STEER_SPEED_MIN_FACTOR` = 1.0). Left alone in M1 — it is
   a handling decision, and the pipeline exists precisely to evaluate it afterwards.

### Reload-safety constraint (easy to violate here)

`Track` lives inside `Game`. `game.h` and `hotreload.h` forbid any pointer into module code or static
data from anything reachable from `Game`. **Track/car identifier strings must be fixed `char[N]` arrays,
never `const char *`.** Heap arrays are fine — `Track.nodes` already is one.

### Unknowns / assumptions stated

- Validation runs on a GPU-capable Windows desktop (raylib needs a GL context). Assumed; `mk validate`
  is deliberately *not* added to the headless `verify` target.
- The uncommitted working-tree changes (dev_scenario +264, handling_tests +254, telemetry schema
  additions, `REPLAY_CAPACITY_TICKS` 7200→14400, parking lot 200×150→400×300) are assumed to land
  before this work starts. The 14400-tick replay capacity (120 s) is in fact required by Phase 4 —
  an out-lap + timed lap exceeds 60 s.

---

## Proposed pipeline architecture

```
tools/validation/run_suite.py            ← orchestration, suite aggregation, regression diff
        │ invokes once per car
        ▼
drifty.exe --validate-lap --car ID --out DIR        (one deterministic process)
        │
        ├── car_roster.c ──────────► VehicleSpec          (enumeration + configuration)
        ├── track.c (chicane_v1) ──► Track + Checkpoint[] (scene load + start pose)
        │
        └── fixed-tick loop @120 Hz
                ├── ai_driver.c ────► Input               ← writes ONLY Input; everything else const
                ├── game_fixed_update()                   ← unchanged path: auto-trans, physics,
                │        │                                    checkpoints, collision
                │        └── replay_record(Input)         ← AI run is replayable for free
                ├── every 2nd tick (60 Hz):
                │        ├── telemetry_write_row()  ──────► telemetry.csv
                │        └── render + LoadImageFromScreen ─► ffmpeg stdin ─► run.mp4
                └── on completion:
                         ├── validation_metrics.c  ───────► summary metrics
                         └── run_report.c          ───────► run.json  (PASS/FAIL + reason)
```

The audit boundary is the `ai_driver_update()` signature: every argument except `Input *out` is `const`.
The compiler, not a convention, prevents the AI from touching a transform, a velocity, or a wheel state.

---

## Phase 1 — Chicane track, runoff, and ordered checkpoints

**Objective.** A closed chicane circuit with an unambiguous start/finish, an ordered required-checkpoint
sequence, a runoff band that makes "off track" distinct from "hit the wall", and checkpoint state that
telemetry can record.

**Files.** `src/world/track.h/.c`, `src/world/collision.c`, `src/game/game.c` (checkpoint call site),
`tests/scenarios/gameplay_tests.c`.

**Retain.** `TrackNode`, the centerline distance helpers, `segments_intersect()`, the parking-lot mode
(free-drive and existing tests still use it), the swept-capsule collision algorithm.

**Change.**
- `TrackNode` gains `float runoffHalfWidthM`. `Track_SurfaceAt()` returns the node surface within
  `halfWidthM`, a runoff surface (grass) between `halfWidthM` and `runoffHalfWidthM`, and
  `offTrackSurfaceId` beyond. `collision_resolve_track()` builds barriers at `runoffHalfWidthM`.
- `track_update_checkpoints()` is rewritten against an explicit `Checkpoint[]` and returns an event:
  ```c
  typedef struct { Vector2 centerM, normalUnit; float halfWidthM; bool required; } Checkpoint;
  typedef struct { bool crossed; int index; bool outOfOrder; bool lapCompleted; float lapTimeS; }
      TrackCheckpointEvent;
  ```
  It tests **every** checkpoint each tick so out-of-order crossings are detectable, and applies the
  existing forward-only dot-product rule against the checkpoint's own normal.
- `Track` gains `Checkpoint *checkpoints; int checkpointCount; char id[32]; char version[16];` — fixed
  char arrays, per the reload-safety constraint.
- `game.c` records the returned event into the frame's telemetry context instead of discarding a `bool`.

**Create.**
- `track_load_chicane(Track *)` — a closed loop of roughly 640 m:
  - main straight ~180 m, half-width 8 m, runoff 12 m, start/finish gate at its midpoint;
  - a **left-right-left chicane** on that straight: ~25 m lateral offset over ~60 m, half-width 6 m,
    runoff 8 m — tight enough to require real braking, turn-in and direction change;
  - two connecting curves (~40 m radius) closing the loop.
  At an achievable ~23 m/s mean that is ~28 s/lap; out-lap + timed lap ≈ 60 s ≈ 3600 frames at 60 fps.
  8 required checkpoints: start/finish, chicane entry, both apexes, chicane exit, and one per curve.
- `track_start_pose(const Track *, Vector2 *posM, float *headingRad)`.
- `game_spawn_on_track(Game *, const Track *)` — resets the vehicle onto the start pose rather than the
  world origin.
- `TRACK_VERSION "chicane_v1"` plus a geometry hash (FNV-1a over the node/checkpoint arrays) so a
  silent geometry edit is visible in every run JSON.

**Remove/deprecate.** Nothing. The parking lot stays for free-drive.

**Dependencies.** None.

**Risks.**
- *Ribbon collision at curve joints.* Per-segment straight barriers form a concave polyline on the outer
  edge; the capsule can catch on a joint and receive a spurious impulse. **Prototype P4 below.**
- *Half-width choice.* Too tight and the AI fails for track reasons; too wide and the chicane tests
  nothing. Tune against the AI prototype, not by eye.

**Validation.** Extend `scenario_checkpoint_lap` in `gameplay_tests.c`: ordered traversal advances;
skipping a required checkpoint does not complete a lap; a reverse crossing does not advance; an
out-of-order crossing is reported and does not advance; lap wrap resets `nextCheckpoint` and records
`lastLapTimeS`. New `scenario_track_runoff`: a point inside `halfWidthM` is asphalt, between the two
widths is grass, beyond `runoffHalfWidthM` is off-track, and a barrier contact occurs only at
`runoffHalfWidthM`.

**Done when.** `drifty_tests --scenario checkpoint-lap` and `--scenario track-runoff` pass; the chicane
loads in the running game and is drivable by hand; `mk regression` shows zero baseline diff (nothing in
this phase touches the default no-track path).

---

## Phase 2 — Physical drivetrain layout, and the car roster

**Objective.** `drivetrainLayout` and `frontTorqueSplit` change how the car drives. A 6-car roster spans
RWD/FWD/AWD and is enumerable by the pipeline.

**Files.** `src/physics/drivetrain.h/.c`, `src/physics/physics.c`, `src/physics/auto_transmission.c`,
new `src/game/car_roster.h/.c`, `Makefile` (`GAME_SRCS`), `tests/scenarios/physics_tests.c`,
`tests/test_commands.c` (`--list-cars`, `--generate-roster`).

**Retain.** The engine torque curve, gearing, rev limiter, engine-braking fade, LSD bias maths, and
`drivetrain_integrate_wheel()` — all unchanged in behaviour.

**Change.**
- `drivetrain_calculate_torques()` takes `const float omegaRadS[WHEEL_COUNT]` and
  `const float tireReactionTorqueNm[WHEEL_COUNT]` instead of the two rear-only pairs.
- Extract the existing rear LSD logic into a reusable
  `apply_differential(mode, omegaL, omegaR, reactL, reactR, axleTorqueNm, bias, preloadNm, &tL, &tR)`
  and call it per **driven** axle.
- Torque split: `frontShare = FWD ? 1.0 : AWD ? clamp(frontTorqueSplit,0,1) : 0.0`; driveline torque is
  divided `frontShare` / `1-frontShare` across axles, then through the per-axle differential.
- `drivetrain_engine_rpm()` takes the mean omega of the **driven** wheels (all four for AWD).
- `auto_transmission.c` reads rpm through the same path.

**Create.** `src/game/car_roster.h/.c` — pure functions of an index, raylib-free and I/O-free, following
the `car_corpus.h` precedent:
```c
int         car_roster_count(void);                       /* 6 */
bool        car_roster_spec(int index, VehicleSpec *out);  /* pure */
void        car_roster_id(int index, char *buf, size_t cap);   /* "rwd_grip" — fs-safe */
const char *car_roster_display_name(int index);
uint32_t    car_roster_spec_hash(int index);
```

| id | layout | seeded from | intended character |
|---|---|---|---|
| `rwd_grip` | RWD | Track Predator | high grip, moderate power, LSD — the reference |
| `rwd_power` | RWD | Pro D1GP | high power, lower rear μ — power oversteer available |
| `fwd_light` | FWD | Shoebox | light, low power — understeer-limited |
| `fwd_hot` | FWD | new (Touge Hero mass class) | more power — torque steer, lift-off rotation |
| `awd_rally` | AWD | Rally Devil | 50/50 split, loose-surface bias |
| `awd_gt` | AWD | Track Predator + AWD | front-biased split, high grip |

Export to `data/vehicles/roster/*.txt` in the existing tuning-profile format via
`drifty_tests --generate-roster`, with a round-trip scenario, exactly as `--generate-corpus` does.

**Remove/deprecate.** Nothing.

**Dependencies.** None (parallel with Phase 1).

**Risks.**
- *Regression on the 8 committed baselines.* This touches the core torque path. Mitigated by the
  bit-identity requirement below.
- *AWD centre-differential fidelity.* M1 uses a fixed torque split, not a modelled centre diff. Stated
  explicitly so nobody assumes otherwise.
- *FWD steering-axle drive torque* newly couples drive force into the steered wheels. The force rotation
  in `physics.c:807` already applies each front wheel's own steer angle, so this should fall out — but
  it is the most likely place for a sign error.

**Validation.**
- **Bit-identity gate:** with `drivetrainLayout = 0`, all 8 baselines in `tests/baselines/` must compare
  **byte-identical** via `mk regression`. Any diff means the refactor is wrong, not that the baseline
  moved.
- New `scenario_drivetrain_layout`: same spec, same `power-oversteer` script, three layouts. Assert RWD
  produces the largest positive rear-minus-front slip-angle differential, FWD the smallest (or negative),
  AWD in between; assert front wheels receive zero drive torque under RWD and nonzero under FWD/AWD.
- `scenario_roster`: every entry passes `vehicle_spec_is_valid()`; ids are unique and filesystem-safe;
  the profile export round-trips.

**Done when.** `mk regression` is clean, the three layout scenarios pass, and
`drifty.exe --list-cars` prints 6 rows.

---

## Phase 3 — Remove drift scoring

**Objective.** The drift-specific gameplay layer is gone. Nothing that remains exists solely to serve it.

**Files.** Delete `src/game/scoring.c/.h`. Edit `src/game/game.h/.c`, `src/physics/vehicle.h`,
`src/physics/physics.c`, `src/render/render_hud.c`, `src/render/render.c`, `src/core/config.h`,
`Makefile` (`GAME_SRCS`), `tests/scenarios/gameplay_tests.c`.

**Remove.**
- `scoring.c/.h` entirely; `derived.scoringDrift` (`vehicle.h:231`, `physics.c:1010`).
- `Game.driftScore`, `bestScore`, `driftTimeS`, `comboMultiplier`, `comboTimerS`.
- `persistence_load_score()` / `persistence_save_score()` and the `%APPDATA%/drifty/bestscore.txt` file.
- Config constants: `MIN_DRIFT_SPEED_MPS`, `MIN_DRIFT_ANGLE_RAD`, `MIN_REAR_SLIP_RAD`,
  `MIN_DRIFT_YAW_RATE_RADS`, `SPIN_CUTOFF_RAD`, `SCORE_BASE_RATE`, `SCORE_SPEED_REF_MPS`,
  `COMBO_GRACE_S`, `MAX_VALID_SCORE`, `DRIFT_ZOOM_REF_RAD`, `CAMERA_ZOOM_RANGE`.
- The score HUD card and DRIFT! callout (`render_hud.c:179-196`, `:435`); drift-driven camera zoom.
- Scoring scenarios in `gameplay_tests.c`: `scoring-accumulation`, `scoring-combo-sweep`,
  `scoring-determinism`, `crash-scoring-interaction`.

**Retain deliberately.**
- `derived.physicallySliding` (`physics.c:1006`) — a **physics** output (`maxFrictionUsage >= 0.98`),
  not a gameplay rule. Tire smoke and the screech in `audio.c` retarget to it.
- `crashLockoutTimerS` — repurposed from a scoring lockout to the collision-event edge that the
  `collisions` metric counts.
- `Game.track.lap` / `lapTimerS` / `lastLapTimeS` — now the primary loop, not a side channel.

**Change.**
- `game.c:468` particle spawn condition: `derived.scoringDrift` → `derived.physicallySliding &&
  derived.speedMps > 5.0f`.
- `game.c:457` results trigger: `RESULTS_TARGET_LAPS` becomes a session lap target rather than a
  drift-run terminator.
- Spin / excessive-yaw detection moves out of the simulation entirely into the metrics layer (Phase 5),
  computed from telemetry columns — see the definition there.

**Dependencies.** None, but sequence it after Phase 1 so the HUD edit and the lap HUD edit land together.

**Risks.** `gameplay_tests.c` currently has ~1766 lines with several scoring scenarios; removing them
mechanically may take related lap/collision assertions with them. Read each scenario before deleting.

**Validation.** Full `drifty_tests` run passes with the scoring scenarios removed and no others broken;
`mk regression` clean (scoring was already excluded from the state checksum, so telemetry cannot move);
`grep -r "driftScore\|scoringDrift\|comboMultiplier" src/ tests/` returns nothing.

**Done when.** The above greps are empty and the suite is green.

---

## Phase 4 — AI driver

**Objective.** One repeatable baseline driver that completes a clean lap in all six cars, using only
steer/throttle/brake, with a structurally auditable separation from the vehicle.

**Files.** New `src/game/ai_driver.h/.c`; `src/game/game.h` (an `AiDriver` block of plain value data),
`src/game/game.c` (the input substitution point), `Makefile` (`GAME_SRCS`), new
`tests/scenarios/ai_tests.c`.

**Create.**
```c
typedef struct {                 /* pure config, identical for every car */
    float lookaheadBaseM, lookaheadSpeedS;
    float corneringGripFraction;      /* < 1.0: drive conservatively */
    float brakeGripFraction;
    float steerGainP, steerGainD;
    float speedGainP;
} AiDriverConfig;

typedef struct { int targetNodeIndex; float prevCrossTrackErrorM; } AiDriverState;

void ai_driver_update(const AiDriverConfig *cfg, AiDriverState *s,
                      const Track *track, const VehicleState *vs,
                      const VehicleDerived *vd, const VehicleSpec *spec,
                      Input *out, float dt);
```
**Every argument except `Input *out` and the driver's own scratch state is `const`.** That is the
auditable boundary, enforced by the compiler rather than by review.

Algorithm — pure pursuit with a curvature speed target:
1. Nearest centerline segment → cross-track error and arc position.
2. Lookahead point at `L = lookaheadBaseM + lookaheadSpeedS · speed` along the centerline.
3. Steering: `κ = 2·sin(α)/L`, `δ = atan(κ · wheelbaseM)`, normalized by `spec->maxRoadWheelAngleRad`
   into `out->steer ∈ [-1,1]`, plus a D term on cross-track error. **`+1` is LEFT** (`input.h:31`).
4. Speed target: maximum centerline curvature over the next few seconds gives
   `v_target = sqrt(corneringGripFraction · μ_lat · g / κ_max)`, then a braking-distance constraint
   back-propagated from the tightest upcoming point using `brakeGripFraction · μ_long · g`.
5. `err = v_target − speed`; `throttle = clamp(speedGainP·err, 0, 1)`,
   `brake = clamp(−speedGainP·err, 0, 1)`. Never both nonzero.
6. `out->handbrake` and every one-shot are **never written**.

μ comes from `spec->tireMuLatFront/Rear` and the surface table, so the target speed adapts per car from
data the car itself publishes — no privileged state.

**Change.** In `game_fixed_update()`, add the AI substitution next to the existing
`dev.scenarioRunning` branch, **before** `replay_record(&game->replay, &tickInput)`. The AI's inputs are
therefore recorded in the replay buffer exactly like a human's, so any AI run is replayable and failure
bundles capture it with no extra work.

**Dependencies.** Phase 1 (needs `Track` centerline), Phase 2 (needs the roster to tune against).

**Risks — this is the highest-risk phase.**
- *A car cannot complete the lap.* This is the correct outcome if the car has a genuine handling defect.
  The plan is explicit: **do not tune the AI per car.** One `AiDriverConfig` is shared; a test asserts it.
  If `fwd_light` cannot make the chicane, the failure and its evidence are the deliverable.
- *Conservative tuning hides differences.* `corneringGripFraction` too low and every car crawls round
  identically. Target ~0.80 initially and check that lap times still spread meaningfully across the six.
- *Pure pursuit oscillates at low lookahead.* Standard; tune `lookaheadSpeedS` and the D gain.
- *Understeer at the limit* makes the pure-pursuit target unreachable and the controller saturates
  steering. Acceptable — that *is* the car's weakness being exposed, and telemetry will show it.

**Validation (`tests/scenarios/ai_tests.c`, headless, no video).**
- `ai-input-only`: run 3600 ticks and assert `handbrake == 0`, no one-shot ever set, and
  `throttle · brake == 0` on every tick.
- `ai-no-privilege`: assert the AI writes nothing outside `Input` — verified by running with the AI and
  with a replay of its recorded input timeline and requiring **identical `stateChecksum`** at every tick.
  This is the strongest available proof that the AI has no side channel.
- `ai-roster-laps`: every car is driven with the same `AiDriverConfig` bytes (snapshotted before
  the roster loop and re-checked after each car), **and** each of the 6 cars completes the
  out-lap + timed lap within the tick budget. Planned here as two scenarios
  (`ai-uniform-config` and `ai-completes-lap`); they landed as one, because both assertions
  need the same roster loop and splitting them would run all six cars twice to assert less.
- Determinism: two runs of the same car produce identical checksums.

**Done when.** All five scenarios pass for all six cars, headless, with no per-car AI tuning.

---

## Phase 5 — Telemetry, metrics, and machine-readable run output

**Objective.** Each run emits a 60 Hz CSV and a `run.json` carrying identity, configuration, checkpoint
events, summary metrics, and an explicit pass/fail with a reason.

**Files.** `src/game/telemetry.h/.c`, `tests/support/simulation_fixture.c`, new
`src/game/validation_metrics.h/.c`, new `src/game/run_report.h/.c`, new `docs/VALIDATION_SCHEMA.md`.

**Retain.** The entire existing `TelemetryRow` schema and column order — the header comment already
establishes "append, never rename" as the convention, and `tests/baselines/` depends on it.

**Change — append to `TelemetryRow`:**
`checkpointIndex`, `lapIndex`, `lapState` (0 out-lap / 1 timed / 2 complete / 3 aborted),
`checkpointEvent` (0 none / 1 in-order / 2 out-of-order / 3 lap-complete), `collisionEvent`,
`distanceToCenterlineM`, `onTrack`, and the two missing per-wheel slip columns (FR and RR — only FL and
RL are written today), so four-wheel diagnosis is possible.

Everything else on the requested telemetry list is **already present**: time, tick, position, heading,
`velocityLongitudinal/Lateral`, speed, yaw rate, steering angle, rpm, gear, per-axle slip angle and slip
ratio, wheel omegas, normal loads, pure and ellipse-limited forces, friction usage, locked flags, drive
torque, front/rear brake torque, handbrake torque, total body force, yaw torque, sideslip, load transfer,
filtered/solved longitudinal accel, lateral accel, aero drag, rolling resistance, all four driver inputs,
per-wheel surface id, and the state checksum. **Not available without invasive physics changes:**
individual engine output torque per axle after the differential is exposed via
`derived.differentialTorqueNm[2]` for the rear only — generalizing it is a Phase 2 side effect worth
taking; true suspension travel/roll angle does not exist (the model is planar) and is out of scope.

**Create — `validation_metrics.c`**, a pure function over the accumulated run:
`lap_time_s`, `max/mean/median/min_moving_speed_mps`, `p05`/`p95` speed, `time_below_5mps_s`,
`checkpoints_passed`, `checkpoints_missed`, `out_of_order_events`, `collisions`, `off_track_events`,
`off_track_time_s`, `spin_events`, `max_abs_sideslip_rad`, `max_yaw_rate_rad_s`, `max_friction_usage`,
`time_at_friction_limit_s`, `max/min_long_accel_mps2`, `max_abs_lat_accel_mps2`.

Two definitions that replace the deleted classifier, both interval-based so they cannot be inflated by
sampling rate:
- **Spin event** — a contiguous interval with `|bodySideslipRad| > 1.48 rad` lasting ≥ 0.25 s while
  `speed > 2 m/s`. Counted as intervals, not ticks.
- **Off-track event** — a contiguous interval where all four wheels report a non-asphalt surface for
  ≥ 0.1 s.
- **Collision** — a rising edge of `crashLockoutTimerS`.

**Create — `run_report.c`**, writing `run.json` (hand-rolled `fprintf` with escaping, following
`failure_bundle.c`):
```json
{ "schema_version": "1.0.0",
  "run_id": "20260807-143201-chicane_v1-rwd_grip",
  "result": { "status": "PASS", "failure_reason": null },
  "car":   { "id": "rwd_grip", "display_name": "...", "drivetrain": "RWD",
             "mass_kg": 1220.0, "spec_hash": "a1b2c3d4" },
  "track": { "id": "chicane", "version": "chicane_v1", "geometry_hash": "...",
             "checkpoint_count": 8, "length_m": 642.0 },
  "sim":   { "fixed_hz": 120, "telemetry_hz": 60, "video_fps": 60,
             "build_commit": "...", "build_dirty": false, "final_state_checksum": "..." },
  "lap":   { "out_lap_time_s": 33.10, "timed_lap_time_s": 31.482,
             "checkpoints_passed": 8, "checkpoints_missed": 0, "out_of_order_events": 0 },
  "metrics": { "...": "see above, units in every key or in docs/VALIDATION_SCHEMA.md" },
  "artifacts": { "telemetry_csv": "telemetry.csv", "video_mp4": "run.mp4",
                 "replay": "replay.txt" } }
```

Failure reasons are a closed set: `checkpoint_missed`, `checkpoint_out_of_order`, `stalled`,
`tick_budget_exceeded`, `invalid_state` (non-finite), `video_encode_failed`, `spec_invalid`.

**Artifact layout** (flat, sortable, bulk-globbable):
```
artifacts/validation/<suite_id>/          suite_id = YYYYMMDD-HHMMSS-<track_version>-<commit>
  suite.json
  <car_id>/ run.json  telemetry.csv  run.mp4  replay.txt
artifacts/validation/latest/              copy of the most recent suite
```

**Dependencies.** Phases 1, 3, 4.

**Risks.** Appending columns invalidates nothing, but `compare_telemetry.py` must tolerate a wider
current file than baseline — verify before relying on it.

**Validation.** `scenario_run_report`: a synthetic run produces a `run.json` that parses, carries every
required key, and reports the injected failure reason; metric maths checked against hand-computed values
on a fixed sample array. Extend the existing `telemetry` scenario for the new columns.

**Done when.** A headless run of one car writes a valid `run.json` + `telemetry.csv`, and
`python -c "import json; json.load(open(...))"` succeeds on it.

---

## Phase 6 — Deterministic video capture

**Objective.** Every run yields an H.264 MP4 of the whole attempt, with a diagnostic overlay, produced
from the same deterministic tick loop that produced the telemetry.

**Files.** `src/platform/main.c` (the `--validate-lap` subcommand), new
`src/render/render_validation_overlay.c`, `src/render/render.h`.

**Retain.** The `--capture-scene` pattern (exact `FIXED_DT_S` stepping, never `GetFrameTime`), and the
pixel-art render target chain.

**Create.**
- Frame pump: run 2 fixed ticks → `game_draw(game, 0.0f)` → `LoadImageFromScreen()` → write RGBA bytes
  to an `ffmpeg` pipe opened with `_popen` → `UnloadImage`. Because 2 ticks = 1 frame and telemetry is
  also written every 2nd tick, **CSV row *N* and video frame *N* are the same simulation tick.**
  ```
  ffmpeg -y -f rawvideo -pix_fmt rgba -s <W>x<H> -r 60 -i - \
         -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p <out>/run.mp4
  ```
- Diagnostic overlay drawn at native resolution after the pixel-art blit (like the HUD): car id, elapsed
  time, `CP n/m`, speed km/h, lap state (OUT-LAP / TIMED / COMPLETE / FAILED), running pass/fail, and
  steer/throttle/brake bars showing the AI's actual `Input`.

**Dependencies.** Phases 1, 4, 5.

**Risks.**
- *Framebuffer orientation.* raylib's `LoadImageFromScreen()` flip behaviour must be **verified**, not
  assumed; add `-vf vflip` only if the prototype shows it is needed.
- *Readback throughput.* 3600 frames × 1280×720×4 B ≈ 13 GB streamed per run (through a pipe, never to
  disk). `glReadPixels` at ~2–5 ms/frame ⇒ roughly 10–20 s per run, ~2 min for the suite. Measure in
  **prototype P3**; if too slow, read back the 640×360 pixel-art target and let ffmpeg upscale, cutting
  readback 4×.
- *ffmpeg absence.* Detect at startup and fail the run with `video_encode_failed` rather than silently
  producing no artifact.

**Validation.** `mk validate CAR=rwd_grip` produces a playable MP4 whose duration equals
`frame_count / 60` within one frame; `ffprobe` reports H.264 and the expected frame count; the overlay is
legible at 1280×720; two runs of the same car produce byte-identical telemetry (the MP4 need not be
byte-identical — the encoder is not required to be deterministic).

**Done when.** One car produces a correct, playable, overlaid MP4 and its frame count matches its
telemetry row count.

---

## Phase 7 — Suite orchestration and the regression loop

**Objective.** One command validates every car and reports programmatically; a second compares a suite
against a previous one.

**Files.** New `tools/validation/run_suite.py`, `tools/validation/compare_runs.py`; `Makefile`;
`src/platform/main.c` (`--list-cars`); `README.md`.

**Create.**
- `drifty.exe --list-cars` — one id per line, so the suite is enumerated from the binary rather than a
  hand-maintained list.
- `run_suite.py`: enumerate → run `--validate-lap --car ID --out DIR` per car → collect `run.json`s into
  `suite.json` (`{suite_id, commit, track_version, total, passed, failed, runs:[...]}`) → copy to
  `artifacts/validation/latest/` → exit nonzero if any run failed. `--cars a,b` and `--no-video` flags
  for fast iteration.
- `compare_runs.py`: diff two suite directories — pass/fail transitions, lap-time deltas, and per-metric
  deltas beyond a tolerance; reuses `tools/telemetry/telemetry_common.py` and delegates the time-series
  diff to the existing `compare_telemetry.py`.
- Makefile: `validate:`, `validate-car: CAR=`, `validate-compare: A= B=`. **Not** added to `verify` or
  `ci` — those are headless and this needs a GL context. Documented as such.

**Dependencies.** Phases 5 and 6.

**Risks.** Suite runtime creep. Keep `--no-video` fast enough (<30 s for six cars headless) that the
inner loop stays usable.

**Validation.** `mk validate` on a clean tree produces six run directories and a `suite.json`; deliberately
break one car's spec and confirm exit status is nonzero, `suite.json` names the failing car, and its
artifacts still exist.

**Done when.** The rapid-iteration loop works end to end: change a parameter → `mk validate` →
read `suite.json` → `compare_runs.py` against the previous suite → open an MP4 only when needed.

---

## Prototypes before the larger refactors

Small, throwaway, run in this order. Each answers one question that would otherwise be discovered late.

| | Prototype | Question it answers | Blocks |
|---|---|---|---|
| **P1** | Pure-pursuit controller against the *existing* parking-lot perimeter, headless, default car only | Does the control law hold a line at racing speed at all? | Phase 4 |
| **P2** | Refactor `drivetrain_calculate_torques()` to the 4-wheel signature with `frontShare` hardwired to 0 | Is the refactor bit-identical for RWD? | Phase 2 |
| **P3** | 600-frame `LoadImageFromScreen` → ffmpeg pipe, timed | Is full-resolution readback fast enough, and is the image upright? | Phase 6 |
| **P4** | Drive the chicane by hand with collisions on, logging every barrier impulse | Do curve-joint barriers produce spurious impulses? | Phase 1 geometry |

---

## Milestone 1 acceptance criteria

**Per car — all six of `rwd_grip`, `rwd_power`, `fwd_light`, `fwd_hot`, `awd_rally`, `awd_gt`:**

1. Enumerated by `drifty.exe --list-cars` and selectable by `--validate-lap --car ID`.
2. Driven exclusively through the `Input` struct — proven by the `ai-no-privilege` scenario, which
   requires the recorded input replay to reproduce identical `stateChecksum` at every tick.
3. Traverses all 8 required checkpoints in order on the timed lap; `checkpoints_missed == 0` and
   `out_of_order_events == 0`.
4. Completes a valid lap and produces a finite `timed_lap_time_s > 0`.
5. Produces a playable H.264 MP4 covering the whole attempt, with the diagnostic overlay.
6. Produces a `telemetry.csv` whose row count equals the MP4 frame count, and a `run.json` that parses
   and carries `schema_version`, `car.id`, `car.spec_hash`, `track.version`, `track.geometry_hash`,
   `run_id`, `sim.build_commit`, `result.status`, and the full metrics block.
7. Re-runnable through `mk validate` after any handling change, with `compare_runs.py` producing a
   metric-level diff against the previous suite.

**Suite level:**

8. `mk validate` exits 0 when all six pass and nonzero otherwise, with `suite.json` naming every failure
   and its `failure_reason`.
9. Failed runs still produce their MP4, CSV, `run.json` and a failure bundle — evidence is never
   discarded.
10. The AI uses one `AiDriverConfig` for all six cars, asserted by `ai-roster-laps`. **A car that
    cannot complete the lap is a FAIL with diagnostic evidence — never a reason to weaken the driver,
    widen the track, or special-case that car.**
11. `mk regression` remains clean against `tests/baselines/`: the drivetrain refactor is bit-identical
    for RWD, and no drift-scoring removal touched a checksummed field.
12. `grep -r "driftScore\|scoringDrift\|comboMultiplier" src/ tests/` returns nothing.

---

## After Milestone 1 (deliberately secondary, not designed here)

- **M2 — handling pass.** Use the pipeline as the instrument: re-enable steering speed-sensitivity,
  implement aero downforce (the params already exist), retune the roster, and judge every change by the
  suite diff rather than by feel.
- **M3 — race presentation.** Lap/sector HUD, best-lap and delta display, start lights, in-game car
  selection, results screen. Vsync vs. `SetTargetFPS` belongs here.
- **M4 — track library.** A second and third circuit, and only then a decision about whether a track
  file format is worth its parser.
- **M5 — opponents.** The AI driver is already a controller behind the player input interface, so
  opponents are a multiplicity problem (multiple `VehicleState`s in `Game`) rather than an AI problem.
  That multiplicity is the one thing here expensive to retrofit — but it is not forced now, because
  nothing in Milestone 1 assumes a single vehicle beyond `Game` holding one.

---

## Verification of the plan as a whole

```bash
mk verify && mk validate && python tools/validation/compare_runs.py artifacts/validation/latest <prev>
```

`mk verify` proves the physics did not regress; `mk validate` proves every car still laps the chicane and
produces its artifacts; `compare_runs.py` proves the handling change you just made did what you expected
and nothing else.
