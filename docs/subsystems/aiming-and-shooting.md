# Aiming and shooting (overview)

Module: [`aiming.py`](../../subsystems/aiming.py)

## Concepts

- **`ShooterAimingTable`** — Stores **distance → launcher RPS** and **distance → hood rotations** lookup tables with linear interpolation. Optional distance → time-of-flight samples exist for logging or overrides.
- **`get_aiming_parameters(...)`** — Combines robot pose, **field-relative velocity**, and real goal pose to produce a **virtual goal** so the shot compensates for robot motion (**shooting on the move**, SOTM).
- **`time_of_flight_trajectory(...)`** — Estimates ball flight time from LUT-derived exit speed and launch angle; used for lead calculations.

## Tunables

Constants at the top of `aiming.py` (for example **`EXIT_VELOCITY_MPS_PER_RPS`** and **`LAUNCH_ANGLE_RAD_PER_HOOD_ROTATION`**) connect mechanism units to physics approximations. Tune with mentor supervision using field data.

## Where it runs

- **`Superstructure.periodic()`** calls into aiming when goals are **`LAUNCH`** or **`AIM*`** and suppliers/tables are wired.
- Individual subsystems (turret, hood, launcher) consume **setpoints** produced by that pipeline.

## Deep dive

For the full pipeline, tuning procedure, and units table, read:

- [launch-on-the-move.md](launch-on-the-move.md)

Turret angle control details:

- [turret-rotate-to-goal.md](turret-rotate-to-goal.md)
