# Controller Bindings

This document outlines all controller bindings defined in `robot_container.py`.

---

## Controllers

| Controller | Port | Role |
|------------|------|------|
| **Driver Controller** | 0 | Driving and intake |
| **Function Controller** | 1 | Scoring, superstructure, climber, turret, hood |

---

## Driver Controller (Port 0)

### Default Command
- **Field-centric drive** (always active unless overridden)
  - Left stick Y → Velocity X (forward/backward)
  - Left stick X → Velocity Y (strafe left/right)
  - Right stick X → Rotational rate (turn)

### Buttons

| Button | Action | Trigger Type | Description |
|--------|--------|--------------|-------------|
| **Left Bumper** | Robot-centric drive | `whileTrue` | Overrides field-centric; drives relative to robot heading |
| **Right Bumper** | Intake | `whileTrue` / `onFalse` | While held: INTAKE. On release: STOP |
| **A** | Brake | `whileTrue` | Swerve brake; wheels lock in current orientation |
| **X** | Point wheels | `whileTrue` | Points wheels in direction of left stick (Y, X) |
| **Start** | Seed field-centric | `onTrue` | Resets robot heading for field-centric orientation |

---

## Function Controller (Port 1)

### Buttons

| Button | Action | Trigger Type | Description |
|--------|--------|--------------|-------------|
| **Y** | Turret → HUB | `onTrue` | Turret rotates to hub target |
| **X** | Turret → DEPOT | `onTrue` | Turret rotates to depot target |
| **B** | Turret → OUTPOST | `onTrue` | Turret rotates to outpost target |
| **A** | — | — | *(no binding)* |
| **Left Bumper** | Feeder INWARD | `whileTrue` / `onFalse` | While held: feeder runs inward. On release: feeder STOP |
| **Right Bumper** | Launcher SCORE | `whileTrue` / `onFalse` | While held: launcher SCORE. On release: launcher IDLE |
| **POV Up** | Climber EXTEND | `onTrue` | Climber extends |
| **POV Down** | Climber STOW | `onTrue` | Climber stows |

### Triggers (Analog)

| Trigger | Threshold | Action | Trigger Type | Description |
|---------|-----------|--------|--------------|-------------|
| **Left Trigger** | > 0.75 | Manual turret (Right stick X) | `whileTrue` | Manual turret rotation while held |
| **Left Trigger** | > 0.75 | Manual hood (Right stick Y) | `whileTrue` | Manual hood rotation while held |
| **Right Trigger** | > 0.75 | Turret → NONE | `onTrue` | Disables turret goal; returns to no target |

---

## Notes

- **Intake** (driver right bumper), **Feeder** (function left bumper), and **Launcher** (function right bumper) bindings are only registered when those subsystems are present on the robot.
- **Turret** and **Hood** bindings are only registered when the turret subsystem is present.
- When the left trigger is held (>75%), both manual turret and manual hood are active; use Right stick X for turret and Right stick Y for hood.
