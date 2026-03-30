# Subsystems

Python packages under [`subsystems/`](../../subsystems/) implement **robot mechanisms** and coordinating layers (`superstructure.py`, `aiming.py`).

## Common pattern: subsystem + IO

Most mechanisms use:

| File | Responsibility |
|------|----------------|
| `__init__.py` | Subsystem class: states, `periodic()`, command factories |
| `io.py` | `SubsystemIO` ABC + `SubsystemIOTalonFX` + `SubsystemIOSim` (+ sometimes replay/logging stub) |

`robot_container.py` picks the IO implementation based on `Constants.CURRENT_MODE` and `has_subsystem()`.

## `StateSubsystem`

Shared behavior for **discrete states** (intake on/off, hood stow/aim, etc.) lives in [`subsystems/__init__.py`](../../subsystems/__init__.py). Mechanism code maps each state to voltages, positions, or closed-loop setpoints.

## Adding a new subsystem (checklist)

1. Create `subsystems/<name>/` with `__init__.py` and `io.py`.
2. Add CAN IDs or parameters to `constants.py`.
3. Instantiate in `robot_container.py` for REAL / SIM / REPLAY as needed.
4. If only one chassis has it, add `has_subsystem("<name>")` gating and update sets in `robot_config.py`.
5. Expose **commands** (`cmd.run`, `runOnce`, or `Subsystem.run`) for anything drivers or autos should run.
6. Document bindings in [../core/controller-bindings.md](../core/controller-bindings.md).

## Topic docs in this folder

Mechanism-specific pages and deep dives are listed in the [documentation index](../README.md).
