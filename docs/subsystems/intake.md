# Intake

Package: [`subsystems/intake/`](../../subsystems/intake/)

## Role

Collects game pieces from the floor. The subsystem exposes **`SubsystemState`** values such as **INTAKE**, **STOP**, and **OUTPUT** (eject). States drive motor outputs through **`IntakeIOTalonFX`** on the robot and **`IntakeIOSim`** in simulation.

## Wiring

- Constructed in `robot_container.py` when `has_subsystem("intake")` is true on the current robot.
- **Driver** bindings: right bumper intake, right trigger output (see [../core/controller-bindings.md](../core/controller-bindings.md)).
- **Superstructure** can set intake state for coordinated auto/teleop modes.

## Files

- `__init__.py` — `IntakeSubsystem`
- `io.py` — `IntakeIO`, hardware, and sim

## Student tips

- Verify **current limits** and **inversion** in `constants.py` and Phoenix configs when replacing motors.
- If the intake stalls, check mechanical binding before raising supply limits.
