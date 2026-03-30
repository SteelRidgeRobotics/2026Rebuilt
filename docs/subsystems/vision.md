# Vision

Package: [`subsystems/vision/`](../../subsystems/vision/)

## What it does

`VisionSubsystem` consumes one or more **`VisionIO`** implementations. On the real robot these are typically **`VisionIOLimelight`** instances for front and back cameras (names and transforms from `constants.py`).

Pose observations are forwarded into the **swerve pose estimator** via the callback passed at construction (`add_vision_measurement` from `SwerveSubsystem`).

## Throttling

`robot.py` adjusts vision throttle while **disabled** (`set_throttle`) to reduce CPU on the RoboRIO between matches.

## Student pointers

- **Transforms** (camera to robot center) must match physical mounting; small errors create aiming bias that changes with distance.
- **Pipeline latency** and exposure settings affect aiming lead—coordinate with the scouting/vision subteam when retuning shooter tables.

## Files

- `__init__.py` — subsystem orchestration
- `io.py` — Limelight IO, inputs dataclass, simulation / replay stubs
