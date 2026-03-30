# Feeder

Package: [`subsystems/feeder/`](../../subsystems/feeder/)

## Role

Moves game pieces between intake and launcher. Like other mechanisms, it uses a PyKit-style **IO layer** (`FeederIOTalonFX`, `FeederIOSim`).

## Control

There is **no dedicated joystick binding** for the feeder in the current `robot_container`; feeder states are set through **`Superstructure` goals** (for example during **INTAKE** or **LAUNCH**) or PathPlanner **named commands**.

When debugging feeder issues, trace from `Superstructure.Goal` mappings in [`superstructure.py`](../../subsystems/superstructure.py), not only the feeder class.

## Files

- `__init__.py` — `FeederSubsystem`, states such as **INWARD** / **STOP**
- `io.py` — IO implementations
