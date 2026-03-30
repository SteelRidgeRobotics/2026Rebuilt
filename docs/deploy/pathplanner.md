# PathPlanner deployment

Directory: [`deploy/pathplanner/`](../../deploy/pathplanner/)

## Layout

- **`autos/`** — `.auto` definitions that reference named commands and paths.
- **`paths/`** — Individual path files used by autos.
- **`settings.json`** — PathPlanner project settings (constraints, robot size, etc.).

These files are deployed with the robot program so **`PathPlannerAuto`** can load them at runtime.

## Robot-side setup

`robot_container._pathplanner_setup`:

1. **Registers `NamedCommands`** — Python `Command` objects callable from PathPlanner by name (for example `"Intake"`, `"Aim to Hub"`, `"Launch"`).
2. **Builds the auto chooser** — Every `.auto` file becomes two dashboard options: normal and **mirrored** for red/blue symmetry.
3. **`set_robot_pose`** — `onChange` handler for the chooser to seed pose when switching autos (implementation details live in `robot_container`).

## Naming contract

Names in PathPlanner must **exactly match** registered strings in Python. If an auto fails to run, open the `.auto` file and compare trigger names to `NamedCommands.registerCommand` calls.

## Student workflow

- Edit paths in **PathPlanner** on a laptop, export/sync into `deploy/pathplanner/`.
- Commit changes with a short message describing the routine (“Add center rush auto”).
- Test on **sim first**, then **tune on carpet** with the real robot—acceleration limits that look fine on paper can slip wheels on FRC carpet.

## See also

- [../core/robot-container.md](../core/robot-container.md)
