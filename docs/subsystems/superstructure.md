# Superstructure

Module: [`superstructure.py`](../../subsystems/superstructure.py)

## Purpose

The superstructure is a **`commands2.Subsystem`** that owns **high-level goals** for the scoring stack. Instead of buttons toggling feeder and launcher independently (and fighting each other), code sets a single **`Superstructure.Goal`**:

- `DEFAULT`, `INTAKE`, `LAUNCH`, `AIMHUB`, `AIMOUTPOST`, `AIMDEPOT`, `STOPLAUNCH`, …

Each goal maps to a tuple of **desired states** (or `None`) for intake, feeder, launcher, hood, and turret. `periodic()` applies those states and, when appropriate, runs unified **aiming** (`get_aiming_parameters`) so turret angle, hood position, and launcher RPS track the **virtual goal** for shooting on the move.

## Why it matters for students

When you add a new behavior that touches more than one motor:

1. Prefer a **new `Goal`** or extending the mapping table over scattering button callbacks.
2. Keep safety interlocks (`override_checks`, sensor gating) in one place so autos and teleop share behavior.

## Wiring

`RobotContainer` passes subsystem references (may be `None` on practice bot), drivetrain for field speeds, **aim pose** supplier (turret-center pose), and `ShooterAimingTable`.

## See also

- [aiming-and-shooting.md](aiming-and-shooting.md)
- [launch-on-the-move.md](launch-on-the-move.md)
- [../core/controller-bindings.md](../core/controller-bindings.md)
