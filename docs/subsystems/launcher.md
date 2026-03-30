# Launcher (flywheels)

Package: [`subsystems/launcher/`](../../subsystems/launcher/)

## Role

Spins **top and bottom** Kraken X44 (or configured) wheels to launch game pieces. Closed-loop **velocity (RPS)** targets come from subsystem state configuration and from **aiming setpoints** when scoring.

## Pose supplier

The launcher receives a **pose supplier** so distance and field-relative logic stay consistent with turret-center referencing when SOTM runs through the superstructure.

## Operator interaction

- **Left trigger** on function controller (when launcher exists) runs **`SCORE`** vs **`IDLE`** directly on the launcher for manual flywheel control alongside superstructure goals—see code in `_setup_controller_bindings`.

## Files

- `__init__.py` — `LauncherSubsystem`
- `io.py` — dual-motor IO, simulation plant

## Tuning

Shooter wheel gains and limits live in `constants.py` (`LauncherConstants`). Distance-based **RPS** setpoints belong in `ShooterAimingTable` ([aiming-and-shooting.md](aiming-and-shooting.md)).
