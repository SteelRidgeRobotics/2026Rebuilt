# Environment and workflow

## Prerequisites

- **Python** version compatible with the RobotPy pin in [`pyproject.toml`](../../pyproject.toml) (`robotpy_version`).
- **WPILib / RobotPy** tooling installed per the official RobotPy documentation for the current season.
- Vendor deps: **Phoenix 6**, **PathPlannerLib** (declared in `pyproject.toml` under `requires`).

Exact install commands change slightly each season; use the RobotPy installer or your team’s documented setup.

## Modes the code cares about

`constants.py` defines `Constants.Mode`:

- **`REAL`** — Code on the RoboRIO during practice or competition.
- **`SIM`** — Physics / WPILib simulation on a developer machine.
- **`REPLAY`** — PyKit log replay (advanced; used when diagnosing past runs).

`CURRENT_MODE` is chosen from `RobotBase.isReal()` so the same codebase branches cleanly in `robot_container.py`.

## Deploying to the robot

1. Connect to the robot network (radio/Ethernet/USB as your team does).
2. Use your team’s standard **`robotpy/deploy`** (or equivalent) so `deploy/` contents (including PathPlanner files) reach the RoboRIO.
3. Confirm **`robot_config.py`** correctly detects **competition vs. practice** robot (MAC address lists, hostname, or `ROBOT_NAME` env var).

After deploy, verify:

- Driver station shows the correct mode and joysticks.
- Dashboard (Elastic) shows pose and subsystem telemetry if enabled.

## Simulation

Simulation builds many subsystems with `*IOSim` classes and can run **FuelSim** (`lib/fuel_sim.py`) for game-piece interaction. Use simulation to test command wiring without touching hardware—still validate on the real robot before an event.

## Logging

On the real robot, `robot.py` configures **PyKit** receivers (for example WPILOG to USB when deployed). Learn where your team stores logs and how to open them in **AdvantageScope** or compatible tools.

## Git habits for student contributors

- Work on a **feature branch**, not directly on the branch you use at comps.
- **Small commits** with clear messages help mentors and leaders review.
- Never commit **secrets** (API keys, personal passwords). FRC projects rarely need them in-repo.
- Before a competition freeze, tag or branch the known-good commit your drive team can fall back to.

See also [guides/making-changes-safely.md](../guides/making-changes-safely.md).
