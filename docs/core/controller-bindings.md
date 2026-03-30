# Controller bindings

All bindings are implemented in [`robot_container.py`](../../robot_container.py) inside **`_setup_controller_bindings`**. If this document disagrees with the code, **trust the code** and update this page.

## Controllers

| Controller | USB order | Role |
|------------|-----------|------|
| Driver | **0** | Drive, intake |
| Function (operator) | **1** | Superstructure goals, manual launcher spin-up, turret presets, climber |

## Driver (port 0)

### Default command (always on unless another command owns the drivetrain)

| Input | Action |
|-------|--------|
| **Left stick Y** | Forward / backward (field-relative when field-centric) |
| **Left stick X** | Strafe |
| **Right stick X** | Turn rate |

### While held / edge

| Control | Trigger style | Action |
|---------|---------------|--------|
| **Left bumper** | `whileTrue` | **Robot-centric** drive (same sticks, robot-relative) |
| **Right bumper** | `whileTrue` / `onFalse` | **Intake** on / **STOP** off — only if `intake` subsystem exists |
| **Right trigger** | axis **> 0.75**, `whileTrue` / `onFalse` | **Intake OUTPUT** (eject) on / **STOP** off — only if intake exists |
| **A** | `whileTrue` | **Swerve brake** |
| **X** | `whileTrue` | **Point wheels** in direction of left stick |
| **Start** | `onTrue` | **Seed field-centric** heading (`seed_field_centric`) |

## Function (port 1)

Bindings depend on which subsystems exist on the current robot (`has_subsystem` / non-`None` instances).

### Launcher present

| Control | Trigger style | Action |
|---------|---------------|--------|
| **Right trigger** | **> 0.75**, `whileTrue` / `onFalse` | **`Superstructure.Goal.LAUNCH`** while held; **`STOPLAUNCH`** on release |
| **Left trigger** | **> 0.75**, `whileTrue` / `onFalse` | Launcher **`SCORE`** while held; **`IDLE`** on release (flywheel / score state on launcher subsystem directly) |

### Turret **and** hood present

| Button | Trigger | Action |
|--------|---------|--------|
| **Y** | `onTrue` | Superstructure **`AIMHUB`** |
| **X** | `onTrue` | **`AIMDEPOT`** |
| **B** | `onTrue` | **`AIMOUTPOST`** |
| **A** | `onTrue` | **`DEFAULT`** goal |
| **Start** | `onTrue` | **`override_checks()`** on superstructure |

### Climber present

| Control | Trigger | Action |
|---------|---------|--------|
| **D-pad Up** | `onTrue` | Climber **`EXTEND`** |
| **D-pad Down** | `onTrue` | Climber **`STOW`** |

## Feeder

There is **no dedicated feeder axis/button** on the function controller in the current `robot_container` wiring. Feeder states are driven by **superstructure goals** (for example intake and launch modes) and by **PathPlanner named commands**.

## Autos vs. teleop

PathPlanner **named commands** can combine goals (e.g. **Launch** may run superstructure launch together with intake). See [../deploy/pathplanner.md](../deploy/pathplanner.md).
