# Hood (shot angle)

Package: [`subsystems/hood/`](../../subsystems/hood/)

## Role

Adjusts **launch angle** via a positional mechanism (Talon FX). States include stow and aiming (`AIMBOT`) where hood position follows the **LUT** from `ShooterAimingTable`.

## Pose supplier

Hood logic can compose with **turret pose** (`get_component_pose(turret_pose)`) for 3D logging—see `robot_container.get_component_poses()`.

## Control

Preset aiming targets come from **superstructure** when goals are `AIMHUB`, `AIMDEPOT`, or `AIMOUTPOST`, and during launch workflows that enable aiming.

## Files

- `__init__.py` — `HoodSubsystem`
- `io.py` — hardware + sim

Tuning workflow ties directly to distance ↔ hood rows in the aiming table ([launch-on-the-move.md](launch-on-the-move.md)).
