# `robot.py` — main robot class

## Role

File: [`robot.py`](../../robot.py)

Class: **`Dwayne`** (extends PyKit **`LoggedRobot`**)

This module is the **entry point** WPILib loads. It:

1. Applies small **logging and stderr filters** (`util`) so overrun spam does not hide real errors.
2. Configures **PyKit** metadata and log sinks depending on `Constants.CURRENT_MODE` (real vs. sim vs. replay).
3. Constructs **`RobotContainer`** — where subsystems and bindings live.
4. Runs **`CommandScheduler.getInstance().run()`** every `robotPeriodic`.
5. Publishes **match time** and **game phase** strings to NetworkTables for **Elastic**.
6. Updates **`Field2d`** pose from the swerve odometry when the drivetrain exists.
7. In simulation, steps **`FuelSim`** from `robot_container.fuel_sim`.

## Life-cycle hooks students touch most

| Method | Typical use |
|--------|-------------|
| `autonomousInit` | Schedule selected auto command |
| `teleopInit` | Cancel hanging auto commands |
| `disabledInit` / `disabledExit` | Throttle vision processing to save CPU |
| `teleopExit` | Optional Elastic notification when FMS attached |

## What not to do here

Avoid piling mechanism logic into `robot.py`. Prefer new subsystems, commands, or superstructure goals so code stays testable and discoverable.

## See also

- [robot-container.md](robot-container.md)
- [util.md](util.md)
