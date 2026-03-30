# Using this repository for future robots and seasons

This document explains how to treat **2026Rebuilt** as a **blueprint** without carrying forward game-specific baggage.

## What is reusable

| Pattern | Why keep it |
|---------|--------------|
| **IO abstraction** (`io.py` per subsystem) | Saves weeks when mechanisms change but command structure does not |
| **`robot_container` composition** | Clear place to wire hardware, sim, and replay |
| **`has_subsystem()` mapping** | Practice vs. competition bots diverge in the real world |
| **Superstructure-style coordinator** | Prevents mechanism conflicts when scoring gets complex |
| **PyKit logging hooks** | Replay and AdvantageScope workflows compound over years |
| **PathPlanner named commands** | Same structure each year; swap autos and paths |

## What to strip or replace each season

- **`constants.py`** field layout (`AprilTagField.k20xx...`), goal poses, and vision transforms.
- **`deploy/pathplanner/`** — all autos/paths for the old game.
- **Mechanism folders** you no longer build (delete or leave stubbed with `has_subsystem` false).
- **Game-specific sim** pieces such as `FuelSim` if the next game does not use them.
- **Generated swerve** after mechanical CAD changes—regenerate rather than inherit blindly.

## How to fork for “next year”

1. **Branch or duplicate** the repo with history intact so you can bisect old bugs.
2. **Archive** this season’s tag (`2026-comp`, event names, etc.).
3. Create a **`YYYYRebuilt`** (or similar) default branch; reset field and PathPlanner content first.
4. Keep **`robot_config.py` MAC lists** accurate; nothing confuses debugging like detecting the wrong robot.
5. Rewrite **student docs** only where behavior changes—preserve the doc *structure* in `docs/` so onboarding stays familiar.

## Teaching continuity

The folder layout under **`docs/`** mirrors code areas on purpose:

- New students learn **where to look** once; next year they open the same paths with updated meat.
- Encourage each cohort to add **one deep-dive** (`docs/subsystems/...`) for anything clever you build.

## Legal and vendor hygiene

Re-verify **vendordeps** and `pyproject.toml` pins when WPILib bumps major versions. Old Phoenix or PathPlanner APIs **will not** translate line-for-line without migration notes.
