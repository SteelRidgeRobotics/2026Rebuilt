# `robot_container.py` — wiring and composition

## Role

File: [`robot_container.py`](../../robot_container.py)

Class: **`RobotContainer`**

This is the **composition root** for the season:

- Selects **max swerve speed** (comp vs. Larry tuner constants).
- Instantiates **drivetrain**, **vision**, and optional mechanisms based on `Constants.CURRENT_MODE` and `has_subsystem()`.
- Builds **`Superstructure`** with intake, feeder, launcher, hood, turret references (some may be `None` on Larry).
- Configures **PathPlanner** `NamedCommands` and **`LoggedDashboardChooser`** autos.
- Defines **swerve requests** (field-centric, robot-centric, brake, point wheels).
- Binds **driver** and **function** Xbox controllers.

## Robot vs. simulation vs. replay

A large `match Constants.CURRENT_MODE` block chooses IO implementations:

- **REAL** — TalonFX IO, Limelight vision IO, generated swerve factory from tuner constants (Larry uses `generated/larry/`).
- **SIM** — Sim IO, `FuelSim` registration, simplified vision.
- **REPLAY** — Stub IO for mechanisms; used with PyKit replay sources.

## PathPlanner integration

`_pathplanner_setup`:

- Registers string **named commands** (`"Intake"`, `"Aim to Hub"`, …) that autos reference.
- Scans `deploy/pathplanner/autos` for `.auto` files and adds each plus a **mirrored** variant using `FlippingUtil` conventions.
- Sets default auto to **`None`** (`cmd.none()`).

## Aiming table

`Superstructure` receives a **`ShooterAimingTable`** instance (see `aiming.py`). Replace or pre-populate this object when you have tuned distance → RPS / hood data from the field.

## Component poses

`get_component_poses()` returns **`Pose3d`** list for turret, hood, and climber for AdvantageScope-style **3D logging**.

## See also

- [constants-and-config.md](constants-and-config.md)
- [controller-bindings.md](controller-bindings.md)
- [../subsystems/superstructure.md](../subsystems/superstructure.md)
- [../deploy/pathplanner.md](../deploy/pathplanner.md)
