# Steel Ridge Robotics — 2026 Robot Code (2026Rebuilt)

This repository is the **Python (RobotPy) command-based** robot program for the team’s 2026 competition robot, nicknamed **Dwayne** in code. It runs on the RoboRIO, integrates **CTRE Phoenix 6** swerve, **PathPlanner** autos, **Limelight** vision, and **PyKit** logging for match and tuning data.

## Synopsis

The software is organized around three ideas students can rely on year after year:

1. **Robot modes** — `robot.py` (`Dwayne`) ties WPILib life cycle (disabled, auto, teleop, simulation) to the global `CommandScheduler` and dashboard output.
2. **Composition** — `robot_container.py` builds the drivetrain, optional mechanisms, and **superstructure** (one place that coordinates intake, feeder, shooter, hood, and turret goals so they do not fight each other).
3. **Hardware abstraction** — Each mechanism uses an **IO layer** (`*_io.py`): real TalonFX implementations on the robot, lightweight sim implementations in simulation, and stub IO for log replay.

Constants, CAN IDs, vision camera transforms, and game-field layout live in `constants.py` and are paired with **`robot_config.py`**, which detects **which physical robot** is running (competition vs. practice “Larry”) and which subsystems exist on that chassis.

## Who this is for

- **Students** learning FRC programming: start with [docs/getting-started/project-tour-for-students.md](docs/getting-started/project-tour-for-students.md) and [docs/README.md](docs/README.md).
- **Mentors** onboarding the team: read [docs/architecture/overview.md](docs/architecture/overview.md) and [docs/guides/using-this-repo-for-future-seasons.md](docs/guides/using-this-repo-for-future-seasons.md).

## Tech stack (at a glance)

| Piece | Role in this project |
|--------|----------------------|
| **RobotPy 2026** | Python bindings for WPILib, `commands2`, AprilTag |
| **Phoenix 6** | Kraken swerve drivetrain, TalonFX mechanisms |
| **PathPlannerLib** | Autos and holonomic path following (`deploy/pathplanner/`) |
| **PyKit** | Logging, AdvantageScope-friendly outputs, NT publishing |
| **Limelight** | Vision pipelines fused into pose (`subsystems/vision/`) |

## Repository layout

| Path | Purpose |
|------|---------|
| `robot.py` | Main robot class; scheduler, logging, simulation hooks |
| `robot_container.py` | Subsystems, bindings, auto chooser, PathPlanner named commands |
| `constants.py` | Field layout, CAN IDs, mechanism gains, vision transforms |
| `robot_config.py` | Robot identity (COMP / LARRY) and `has_subsystem()` map |
| `subsystems/` | Drivetrain, vision, superstructure, mechanisms (each may have `io.py`) |
| `generated/` | Tuner X swerve constants (comp vs. Larry) |
| `deploy/pathplanner/` | Auto and path definitions deployed to the robot |
| `lib/` | Team utilities (Elastic notifications, fuel sim, etc.) |
| `tests/` | PyFRC / pytest entry points |
| `docs/` | **Teaching and reference documentation** (mirrors project areas) |

## Quick links

- [Documentation index](docs/README.md)
- [Environment setup](docs/getting-started/environment-and-workflow.md)
- [Controller bindings](docs/core/controller-bindings.md)

## License and team policy

Follow your team’s rules for competition code, vendor licenses (CTRE, WPILib, PathPlanner), and student safety. When in doubt, ask a lead mentor before sharing credentials or deployment access.
