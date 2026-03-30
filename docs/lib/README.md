# `lib/` — team libraries

## `elasticlib.py`

Publishes **Elastic** dashboard notifications over NetworkTables (`Notification`, `NotificationLevel`, `send_notification`). Used from `robot.py` (for example a friendly message at end of teleop when the FMS is attached).

## `fuel_sim.py`

**FuelSim** simulates fuel/game-piece physics and interaction with the field and robot for **simulation-only** testing. `RobotContainer` registers the robot footprint and pose suppliers; `robot._simulationPeriodic` calls `fuel_sim.update_sim()`.

This is large, third-party-inspired code—when teaching beginners, treat it as a **black box** that makes sim more realistic; dive in only if your project depends on changing sim behavior.

## Adding new shared code

Prefer small modules here for **non-subsystem** helpers that still belong to the team (dashboard glue, math, custom trajectories). Keep robot-agnostic math pure; keep WPILib-facing code thin.
