# Swerve drivetrain

Package: [`subsystems/swerve/`](../../subsystems/swerve/)

## What it is

`SwerveSubsystem` subclasses Phoenix 6 **`SwerveDrivetrain`** and **`commands2.Subsystem`**. It was produced with the **2026 CTRE Swerve Project Generator (Tuner X)** and customized for PathPlanner holonomic control, logging, and pose fusion.

## Key responsibilities

- **Odometry** — tracks field pose from module positions and headings.
- **Requests** — field-centric, robot-centric, brake, and point-wheels requests configured in `robot_container._setup_swerve_requests`.
- **Vision** — `add_vision_measurement` callback consumed by `VisionSubsystem`.
- **Field-relative velocity** — used by shooting-on-the-move math (`get_field_relative_speeds()`); see [launch-on-the-move.md](launch-on-the-move.md).

## Generated constants

Swerve geometry, module offsets, and steer gains live under **`generated/`**. Larry may use **`generated/larry/tuner_constants.py`**. See [../generated/tuner-constants.md](../generated/tuner-constants.md).

## Student pointers

- If the robot **drifts** in auto, check wheel calibrations and vision latency first.
- If **teleop feels wrong**, confirm joystick axes and whether field-centric seed was done (**Start** on driver controller).
- Avoid editing generated swerve files by hand—regenerate from Tuner when mechanical geometry changes.
