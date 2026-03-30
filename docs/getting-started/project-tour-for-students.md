# Project tour for students

This walkthrough assumes you have played with *some* programming (blocks, Python, or Java) but are new to **FRC robot code**. Take it in one sitting, then use the [documentation index](../README.md) as a reference.

## What the robot program actually does

Between matches, the RoboRIO runs your code in a loop, many times per second. WPILib calls methods like `robotPeriodic()` on your robot class. In this project, **`CommandScheduler.getInstance().run()`** runs inside `robotPeriodic()`, so **commands** (drive, shoot, auto routines) get a slice of time every loop.

You do **not** usually write long `while` loops that block the robot. Instead you schedule **commands** that finish quickly or run until cancelled.

## The three files everyone should know

1. **`robot.py`** — The `Dwayne` class connects WPILib modes to logging and creates a **`RobotContainer`**. If someone says “the robot won’t start,” you check deploy and this file first.

2. **`robot_container.py`** — Construct subsystems, wire **Xbox controllers**, register **PathPlanner** autos, and build the **superstructure**. Most day-to-day feature work touches this file or a subsystem.

3. **`constants.py`** — Numbers that describe the field and hardware: CAN IDs, camera positions, PID slots, hub locations. If a value is shared or tuned often, it belongs here (or in a dedicated config module you add).

## “Subsystem” and “IO”

A **subsystem** is a robot part represented as a Python class (`IntakeSubsystem`, `SwerveSubsystem`, …). WPILib’s command scheduler uses subsystems to avoid two commands driving the same motors at once.

Many of our subsystems split into:

- **Logic** — `subsystems/<name>/__init__.py` (states, commands, safety)
- **IO** — `subsystems/<name>/io.py` (talks to TalonFX or a simulator)

On the real robot we construct `IOTalonFX` classes; in simulation we use `IOSim` or lightweight fakes. That swap happens in `robot_container.py` inside a `match Constants.CURRENT_MODE` block.

## Superstructure: why it exists

Several mechanisms must cooperate: you cannot intake and launch blindly without defining who wins. **`Superstructure`** (`superstructure.py`) maps **high-level goals** (intake, aim hub, launch, …) to **states** on intake, feeder, launcher, hood, and turret. Buttons and autos ask the superstructure for a goal; the superstructure updates mechanisms in `periodic()`.

When you add a new scoring mode, you often:

1. Add a `Goal` enum value.
2. Extend the goal-to-state table.
3. Bind a button or PathPlanner named command to `set_goal_command(...)`.

See [subsystems/superstructure.md](../subsystems/superstructure.md).

## Vision and pose

The drivetrain tracks **pose** (x, y, heading on the field). Limelights add **vision measurements** that correct drift. If auto or aiming looks wrong, pose is often the root cause (camera transforms in `constants.py`, latency, or blocked tags).

See [subsystems/vision.md](../subsystems/vision.md).

## Where to make common changes

| I want to… | Start in… |
|------------|-----------|
| Change a button | `robot_container.py` → `_setup_controller_bindings` |
| Add a motor / subsystem | New folder under `subsystems/`, wire in `robot_container.py`, update `has_subsystem()` in `robot_config.py` if practice bot differs |
| Tune shooter / hood | `subsystems/aiming.py` table + [launch-on-the-move.md](../subsystems/launch-on-the-move.md) |
| Add or edit an auto | PathPlanner project under `deploy/pathplanner/`, named commands in `robot_container.py` |
| Change swerve geometry | Phoenix Tuner X project → regenerate `generated/` (see [generated/tuner-constants.md](../generated/tuner-constants.md)) |

## Learning order (recommended)

1. [environment-and-workflow.md](environment-and-workflow.md) — run code on your laptop.
2. [../architecture/overview.md](../architecture/overview.md) — mental model.
3. [../core/controller-bindings.md](../core/controller-bindings.md) — what the drivers actually press.
4. Pick one subsystem folder and read `__init__.py` and `io.py` side by side.

Ask a programming mentor before flashing the RoboRIO or changing `robot_config.py` detection on competition day.
