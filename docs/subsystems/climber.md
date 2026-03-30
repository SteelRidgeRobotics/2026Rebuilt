# Climber

Package: [`subsystems/climber/`](../../subsystems/climber/)

## Role

Extends and stows a climbing mechanism using **position control** on a Talon FX (`PositionVoltage`). States are intentionally minimal in the current code—typically **`STOW`** and **`EXTEND`**—with target rotations from `Constants.ClimberConstants` and per-state configs in `ClimberSubsystem`.

## Availability

Construction is gated in `robot_container.py` with `has_subsystem("climber")`. On the competition robot this may be disabled if the mechanism is not installed—check **`COMP_SUBSYSTEMS`** in `robot_config.py`.

## Operator bindings

Function controller **D-pad up/down** triggers extend/stow when the subsystem exists.

## IO layer

- **`ClimberIO`** — abstract inputs + `set_position`
- **`ClimberIOTalonFX`** — real hardware with configurable TalonFX configuration from `robot_container`
- **`ClimberIOSim`** — simulation using WPILib `DCMotorSim`–style modeling

## Note on folder README

`subsystems/climber/README.md` may describe older patterns (for example servos). Treat **`io.py` and `__init__.py`** as the source of truth for the current season.
