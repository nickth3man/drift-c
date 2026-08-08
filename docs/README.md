# Drifty documentation

Two directories, split by whether a document tells you how the code behaves **now**.

- **[design/](design/)** — current contracts. If one of these disagrees with the code, that is
  a bug in one of them, and worth resolving rather than ignoring.
- **[notes/](notes/)** — historical: evaluations, surveys, and investigations, each dated and
  carrying a header saying what is stale about it. Useful for *why*, not for *what is true*.

Start elsewhere for the basics: [README](../README.md) for what Drifty is and how to build it,
[CONTRIBUTING](../CONTRIBUTING.md) for setup and what must be green before pushing, and
[AGENTS.md](../AGENTS.md) for agent workflow rules.

## design/

| Document | Covers |
| --- | --- |
| [CAR_APPEARANCE_IDENTITY](design/CAR_APPEARANCE_IDENTITY.md) | Stable procedural car identity — `CarAppearanceSpec` in `src/render/car_appearance.h` |
| [CAR_VISUAL_PRIMITIVES](design/CAR_VISUAL_PRIMITIVES.md) | The derived fields on `CarVisual` and the rules that produce them |
| [DYNAMIC_CAR_VISUAL_EFFECTS](design/DYNAMIC_CAR_VISUAL_EFFECTS.md) | `VehicleVisualEffects` — dynamic feedback derived from vehicle state |
| [CAR_SPRITE_ROTATION_STABILITY](design/CAR_SPRITE_ROTATION_STABILITY.md) | Measuring point-sampled rotation stability — `make measure-rotation` |
| [CAR_VISUAL_RGBA_REGRESSION](design/CAR_VISUAL_RGBA_REGRESSION.md) | Pixel-level appearance comparison — `make compare-rgba` |

## notes/

| Document | Why it is here |
| --- | --- |
| [ADVERSARIAL_CRITIQUE](notes/ADVERSARIAL_CRITIQUE.md) | Phase 2 evaluation, 6 Aug 2026. Ratings are stale; §5 still lists open fixes |
| [GAME_INTERACTION_RECORDING_AND_REVIEW](notes/GAME_INTERACTION_RECORDING_AND_REVIEW.md) | Survey of five recording designs. System 3 became `make record`; the rest were not adopted |

## Adding a document

Put it in `design/` if it describes how the code behaves and you intend to keep it true, and
in `notes/` otherwise. Date anything in `notes/` and say in a header what is stale about it —
an undated note is indistinguishable from a current spec after a few months, which is the
problem this split exists to solve. Then add a row above; a document nobody can find from
here is one nobody reads.
