# Learned Racing Line

The AI's target path is no longer the authored centreline. Each validation circuit carries a
**learned racing line**: a per-node lateral displacement found offline by a neural policy searched
with cross-entropy neuroevolution against a friction-limited lap-time model, constrained to pass
every ordered checkpoint.

- Optimiser: `tools/validation/learn_racing_line.py`
- Line data: `src/world/track.c` (`*RacingLineOffsetsM`, `apply_racing_line_controls`)
- Consumer: `src/game/ai_driver.c` (`TrackNode.racingLineM`)
- Test: `drifty_tests --scenario racing-line`
- Artifacts: `artifacts/racing_line/<track>_nn_line.json`

## Why a learned line rather than a hand-authored one

The lap-time-optimal path through a sequence of gates is a trade between path length and
curvature: a shorter line is tighter, a tighter line is slower through its apex, and the balance
depends on the vehicle's friction budget. That is an optimisation problem, not a drafting problem,
and the literature solves it with either trajectory optimisation or learning. This project uses a
learned policy because the same policy generalises across the three authored layouts without
re-deriving geometry by hand.

## Method

**Parameterisation.** A 145-parameter MLP (10 inputs, 12 hidden, tanh, 1 output) maps local track
shape to a lateral offset from the centreline at each of the 170 centreline nodes:

| Input | Meaning |
|---|---|
| `kappa[i]` | signed centreline curvature, so the policy can tell a left-hander from a right-hander |
| `kappa[i+6]`, `kappa[i+14]`, `kappa[i+26]` | look-ahead curvature — turn in before an apex |
| `kappa[i-6]`, `kappa[i-14]` | look-behind curvature — unwind after one |
| `abs(kappa[i])` | corner severity irrespective of direction |
| `halfWidth[i]/8` | how much road there is |
| `sin`, `cos` of lap phase | position around the closed loop |

The output is scaled to the available amplitude and passed through a Hann-weighted circular
moving average (9 nodes). The smoothing matters: the authored circuits have piecewise-constant
curvature, so a curvature-driven policy emits step changes at every curve entry, and a
pure-pursuit driver cannot track a step. Smoothing the parameterisation raised the modelled gain
from ~1.5% to ~5% because the optimiser no longer had to buy smoothness by shrinking the line.

**Search.** Cross-entropy method: sample a population from a diagonal Gaussian, evaluate, refit
the mean and standard deviation to the elite fraction, repeat. Deterministic from `--seed`.
Defaults are 200 iterations × 128 candidates; the current lines were trained with those values at
seed `20260807`.

**Objective.** Minimise modelled lap time subject to constraints:

1. **Lap time** — forward/backward speed-profile pass over the candidate line. Corner speed is
   `sqrt(mu*g/kappa)`, then propagated forward under acceleration and backward under braking.
2. **Grip realism** — the model plans at `0.78 g` lateral, because the driver only ever spends
   `corneringGripFraction` (0.75) of the tyre. Planning at the tyre's full mu produced apexes no
   car on the roster could carry speed through.
3. **Edge margin** — the line stays `2.4 m` from the surface edge: half a car plus the tracking
   error a pure-pursuit driver shows on a curved line. Enforced independently in
   `apply_racing_line_controls`, so the C loader cannot import an illegal line.
4. **Drivability** — the second difference of the offset profile is penalised. A lap-time model
   rewards any kink that shortens the path; a real driver lags it and runs wide.
5. **Curvature budget** — peak curvature may not exceed the centreline's. One line is shared by a
   six-car roster, and corner speed scales as `sqrt(mu/kappa)`: a tighter apex only pays for cars
   with grip to spare. Without this cap the low-grip `fwd_light` lost **3.9 s** on the technical
   circuit while every other car gained. With it, the gain comes from shortening and
   straightening, which every car can use.
6. **Checkpoints** — the line must pass inside every ordered gate. Verified independently in C.

**Storage.** The learned per-node profile is resampled to 48 control offsets and embedded as
static data in `src/world/track.c`, then re-interpolated at load. Resampling costs 0.115 m of
position and 0.002 1/m of curvature against the per-node line — measured, not assumed. Embedding
keeps the simulation deterministic and free of a runtime file dependency.

## Regenerating

```sh
python tools/validation/learn_racing_line.py --track chicane   --iterations 200 --population 128 \
    --output artifacts/racing_line/chicane_nn_line.json
python tools/validation/learn_racing_line.py --track sprint    --iterations 200 --population 128 \
    --output artifacts/racing_line/sprint_nn_line.json
python tools/validation/learn_racing_line.py --track technical --iterations 200 --population 128 \
    --output artifacts/racing_line/technical_nn_line.json
```

Copy `control_offsets_m` from each JSON into the matching array in `src/world/track.c`, rebuild,
then re-run `drifty_tests --scenario racing-line` and `tools/validation/run_suite.py`. The track
geometry hash changes when the line changes, so older runs stay identifiable.

## Results

Modelled lap time (optimiser's own objective, `0.78 g`):

| Track | Centreline | Learned | Gain | Peak curvature (learned vs centre) |
|---|---:|---:|---:|---|
| chicane | 33.481 s | 31.919 s | 4.67% | 0.0485 vs 0.0482 1/m |
| sprint | 30.915 s | 29.386 s | 4.95% | 0.0633 vs 0.0629 1/m |
| technical | 25.798 s | 24.371 s | 5.53% | 0.0733 vs 0.0735 1/m |

Simulated timed lap, `racing-line` scenario — same car, controller, and standing start, driving
the learned line versus the centreline:

| Track | Path length | Centreline lap | Learned lap | Gain |
|---|---:|---:|---:|---:|
| chicane | 659.2 m (vs 690.0 m) | 40.093 s | 38.409 s | 4.20% |
| sprint | 546.3 m (vs 575.2 m) | 37.001 s | 35.759 s | 3.36% |
| technical | 402.8 m (vs 422.9 m) | 31.558 s | 30.308 s | 3.96% |

Full validation suite, six cars × three tracks, learned line versus the pre-optimisation
centreline build: **18/18 PASS, 17 cases faster, 1 unchanged, 0 slower.** Best case
`sprint/fwd_light` −2.008 s; the `technical/fwd_light` case that the curvature budget was added
for went from −3.9 s worse to level.

## Research basis

- **Wang, Yuan, Sun — *Learning Autonomous Race Driving with Action Mapping Reinforcement
  Learning*** (arXiv:2406.14934). Establishes the state/reward shape used here: forward-observation
  points along the centreline as the policy's view of upcoming track, progress along the track as
  the objective, and — the part this implementation borrows most directly — enforcing the tyre
  friction limit as a *constraint on the action* rather than as a reward penalty, because penalties
  produce conservative policies. Our friction-limited speed profile and grip-realistic planning
  level play the same role.
- **Kalaria et al. — *Local NMPC on Global Optimised Path for Autonomous Racing***
  (arXiv:2109.07105). The global/local split this project uses: compute a boundary-constrained
  optimal racing line offline, then track it with a local controller that maximises progress. Here
  the offline stage is the neuroevolution search and the local stage is the pure-pursuit driver.
- **Jung et al. — *Game-Theoretic MPC with Data-Driven Identification of Vehicle Model***
  (arXiv:2106.04094). Confirms the layering — trajectory planner above a low-level follower — used
  for the Indy Autonomous Challenge.
- **`ParsaD23/Racing-Line-Optimization-with-PSO`.** Closest published analogue: sector-wise
  parameterisation of the line between inner and outer track boundaries, with a lap-time cost
  function evaluated per candidate and a population-based search over the sector displacements.
  Our control-point offsets, `lap_time_model_s` cost, and CEM population play the same three roles.
- **`TUMRT/online_3D_racing_line_planning`.** Separates a global racing line from a local planner
  and stores the result as a resampled trajectory — the pattern behind embedding the offsets as
  data rather than solving at load time.
- **`dgnzlz/Capstone_AWS_DeepRacer`.** Reward shaping against a precomputed optimal line,
  `dist_to_racing_line` normalised by track width — the same "distance from the target line,
  scaled by available road" quantity the AI uses as its cross-track error.
- **VDrift `src/ai/ai_car_standard.cpp`.** The production deterministic racing AI this driver
  resembles: racing-line-aware lookahead and curvature-derived speed limits.
