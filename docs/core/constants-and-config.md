# `constants.py` and `robot_config.py`

## `constants.py`

File: [`constants.py`](../../constants.py)

Class: **`Constants`**

Houses **numeric and geometric configuration** used across subsystems:

- **`Constants.Mode`** and **`CURRENT_MODE`** — real vs. sim selection.
- **`FIELD_LAYOUT`** — AprilTag field (this project loads **`k2026RebuiltWelded`**).
- **`CanIDs`** — CAN identifiers for TalonFX devices (and optional PDH ID for PyKit logging).
- Per-mechanism nested classes (**gains**, gear ratios, limits, supply current limits, etc.).
- **Vision** camera names and **Transform3d** from robot origin to each camera.
- **Goal locations** (e.g. hub poses) for aiming.

**Student tip:** When you duplicate a “magic number” in two files, move it here (or a small dedicated module) with a comment about units.

## `robot_config.py`

File: [`robot_config.py`](../../robot_config.py)

Resolves **which physical robot** is running:

- **`Robot` enum** — `LARRY` (test), `COMP` (competition), `UNKNOWN`.

**`detect_robot()`** tries, in order:

1. **MAC address** match against lists you maintain for each robot.
2. **Hostname** substrings (`larry`, `comp`, …).
3. Environment variable **`ROBOT_NAME`**.

At import time, **`currentRobot`** is fixed for the process.

### `has_subsystem(name: str) -> bool`

Returns whether the **current robot** has a mechanism wired and supported in code. `robot_container.py` uses this to skip constructing hardware that would fault on a partial chassis.

**When you add a subsystem** to only one bot:

1. Add the subsystem in `robot_container.py` behind `has_subsystem(...)`.
2. Update **`LARRY_SUBSYSTEMS`** / **`COMP_SUBSYSTEMS`** sets in `robot_config.py`.

## See also

- [robot.md](robot.md)
- [robot-container.md](robot-container.md)
