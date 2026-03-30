# Documentation index

This folder mirrors the **areas of the codebase** so students can jump from a subsystem package to its explanation. Read in order the first time through; later, use this page as a map.

## Start here

| Document | Audience | Contents |
|----------|----------|----------|
| [getting-started/project-tour-for-students.md](getting-started/project-tour-for-students.md) | New programmers | Big-picture walkthrough, vocabulary, where to change things |
| [getting-started/environment-and-workflow.md](getting-started/environment-and-workflow.md) | Everyone | RobotPy, deploy, sim, logs, branch habits |
| [architecture/overview.md](architecture/overview.md) | Students + mentors | Layers: robot → container → subsystems → IO |
| [architecture/command-scheduler-and-subsystems.md](architecture/command-scheduler-and-subsystems.md) | Students | Commands, triggers, default commands, requirements |

## Core modules (repository root)

| Document | Source files |
|----------|----------------|
| [core/robot.md](core/robot.md) | `robot.py` |
| [core/robot-container.md](core/robot-container.md) | `robot_container.py` |
| [core/constants-and-config.md](core/constants-and-config.md) | `constants.py`, `robot_config.py` |
| [core/util.md](core/util.md) | `util.py` |
| [core/controller-bindings.md](core/controller-bindings.md) | Controller code in `robot_container.py` |

## `subsystems/` — mechanisms and coordination

| Document | Package / topic |
|----------|-----------------|
| [subsystems/README.md](subsystems/README.md) | IO pattern, `StateSubsystem`, where to add a mechanism |
| [subsystems/swerve.md](subsystems/swerve.md) | `subsystems/swerve/` |
| [subsystems/vision.md](subsystems/vision.md) | `subsystems/vision/` |
| [subsystems/superstructure.md](subsystems/superstructure.md) | `superstructure.py` |
| [subsystems/aiming-and-shooting.md](subsystems/aiming-and-shooting.md) | `aiming.py`, tables, tuning |
| [subsystems/intake.md](subsystems/intake.md) | `subsystems/intake/` |
| [subsystems/feeder.md](subsystems/feeder.md) | `subsystems/feeder/` |
| [subsystems/launcher.md](subsystems/launcher.md) | `subsystems/launcher/` |
| [subsystems/hood.md](subsystems/hood.md) | `subsystems/hood/` |
| [subsystems/turret.md](subsystems/turret.md) | `subsystems/turret/` |
| [subsystems/climber.md](subsystems/climber.md) | `subsystems/climber/` |
| [subsystems/launch-on-the-move.md](subsystems/launch-on-the-move.md) | Deep dive: SOTM pipeline |
| [subsystems/turret-rotate-to-goal.md](subsystems/turret-rotate-to-goal.md) | Deep dive: turret math |

## `generated/`

| Document | Notes |
|----------|--------|
| [generated/tuner-constants.md](generated/tuner-constants.md) | Swerve project from Tuner X; Larry vs. comp |

## `deploy/`

| Document | Notes |
|----------|--------|
| [deploy/pathplanner.md](deploy/pathplanner.md) | Autos, paths, named commands |

## `lib/`

| Document | Notes |
|----------|--------|
| [lib/README.md](lib/README.md) | Team libraries |

## `tests/`

| Document | Notes |
|----------|--------|
| [tests/testing.md](tests/testing.md) | Running tests, simulation checks |

## Guides for the team

| Document | Purpose |
|----------|---------|
| [guides/making-changes-safely.md](guides/making-changes-safely.md) | Checklists before merging or deploying |
| [guides/using-this-repo-for-future-seasons.md](guides/using-this-repo-for-future-seasons.md) | Forking, stripping game-specific pieces, keeping patterns |
