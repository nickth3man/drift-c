# Drifty documentation

Two directories, split by whether a document tells you how the code behaves **now**.

- **[design/](design/)** — current contracts. If one of these disagrees with the code, that is
  a bug in one of them, and worth resolving rather than ignoring.
Historical evaluations and investigations are not part of the current contract; delete or date
them before adding them under `notes/`.

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
| [RACING_LINE_OPTIMIZATION](design/RACING_LINE_OPTIMIZATION.md) | The learned racing line: how the per-node lateral offsets are found and consumed |
| [VALIDATION_SCHEMA](design/VALIDATION_SCHEMA.md) | The `run.json` / `suite.json` contract the validation pipeline emits |

There are no retained historical notes at present.

## Adding a document

Put it in `design/` if it describes how the code behaves and you intend to keep it true, and
in `notes/` otherwise. Date anything in `notes/` and say in a header what is stale about it —
an undated note is indistinguishable from a current spec after a few months, which is the
problem this split exists to solve. Then add a row above; a document nobody can find from
here is one nobody reads.
