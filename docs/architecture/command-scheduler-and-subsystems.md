# Command scheduler and subsystems

WPILib’s **command-based** framework schedules work in small steps. This project uses **`commands2`** from RobotPy.

## Scheduler

`CommandScheduler.getInstance().run()` executes repeatedly from `robot.py` (`robotPeriodic`). Each call:

- Runs **active commands** a bit further.
- Resolves **requirements** so two commands do not control the same subsystem unless you explicitly allow it.

## Subsystems

A **subsystem** implements `Subsystem` and represents hardware (or a logical group). Commands declare `addRequirements(subsystem)` so the scheduler knows conflicts.

**Default commands** run when no other command requires that subsystem. Here, the drivetrain’s default command is **field-centric swerve** from the left and right sticks (`robot_container._setup_controller_bindings`).

## Triggers and buttons

`CommandXboxController` wraps WPILib HID:

- **`whileTrue(command)`** — starts when pressed, cancels when released (pattern used for hold-to-run).
- **`onTrue(command)`** — fires once on rising edge.

`Trigger(lambda: axis > 0.75)` is used for **analog triggers** on Xbox controllers.

## InstantCommand vs. subsystem commands

- **`InstantCommand(lambda: ...)`** — runs a callback once; useful for simple state toggles when you already have imperative APIs like `set_desired_state`.
- **Commands returned by subsystems** — prefer these when you need lifecycle, `finally` behavior, or composition with `alongWith`, `until`, etc.

The codebase uses both; when adding features, prefer subsystem helpers for readability and testing.

## Autonomous

`autonomousInit` schedules whatever **`LoggedDashboardChooser`** selected. Options are populated from PathPlanner `.auto` files plus a short “Basic Leave” fallback. Named commands inside those autos map to superstructure goals (see [../deploy/pathplanner.md](../deploy/pathplanner.md)).

## Practice: trace one button press

1. Find the binding in [../core/controller-bindings.md](../core/controller-bindings.md).
2. Open `robot_container.py` → `_setup_controller_bindings`.
3. Follow the command into `Superstructure` or the subsystem it calls.
4. Read that subsystem’s `periodic()` and IO `set_*` methods.

This loop builds intuition faster than reading abstractions in isolation.
