# Turret

Package: [`subsystems/turret/`](../../subsystems/turret/)

## Role

Rotates the shooter assembly in the horizontal plane so the launcher aims at field targets (hub, depot, outpost headings). Uses **closed-loop position** on a Talon FX with software limits and hysteresis near mechanical center.

## Goals and angles

Subsystem states (for example **HUB**, depot/outpost variants, **MANUAL**) combine with **field-angle setpoints** from the superstructure when SOTM is active. The detailed math for `rotate_to_goal` is documented in:

- [turret-rotate-to-goal.md](turret-rotate-to-goal.md)

## Pose supplier

Like other aiming components, the turret receives a **pose supplier** anchored to robot pose; `util.make_turret_pose_supplier` shifts origin to **turret center** using `Constants.TURRET_OFFSET`.

## Operator presets

Function controller **Y / X / B** selects superstructure aim goals; see [../core/controller-bindings.md](../core/controller-bindings.md).

## Files

- `__init__.py` — `TurretSubsystem`
- `io.py` — `TurretIOTalonFX`, `TurretIOSim`
