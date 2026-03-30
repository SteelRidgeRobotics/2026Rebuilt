# Shooting on the Move & Unified Aiming

This document describes the **current** unified kinematic aiming system: it replaces the old zone-based shooter logic with a single **Virtual Goal + LUT** model so the robot can aim and shoot while moving (SOTM) and while stationary.

---

## Current Architecture

### 1. Drivetrain: Field-Relative Velocity

The turret lead for a moving shot uses the robot’s velocity in **field** frame.

- **Location:** `subsystems/swerve/__init__.py`
- **API:** `SwerveSubsystem.get_field_relative_speeds() -> ChassisSpeeds`
- **Logic:** Cached robot-relative chassis speeds are rotated by the current pose rotation via `Translation2d(vx, vy).rotateBy(pose.rotation())`, yielding field-frame vx, vy (omega unchanged).

### 2. Shooter / Hood: Distance LUT; ToF from Trajectory

RPM and hood angle come from a **distance-based lookup table** with linear interpolation. **Time-of-flight (ToF)** is **computed from velocity and angle** (trajectory), not from a table.

- **Location:** `subsystems/aiming.py` — `ShooterAimingTable` + `time_of_flight_trajectory()`
- **RPM / hood:** Two tables: distance (m) → launcher **RPS**, distance (m) → hood **rotations**. Same as before.
- **ToF definition:** Time from **ball exit** (when the ball leaves the robot) to **impact** at the goal. Used for lead: we aim at \( G_{real} - \vec{V}\times\text{ToF} \). ToF is *not* from command start—it’s from launch to impact.
- **ToF calculation:** For the distance to the real goal we look up RPS and hood, convert to exit velocity (m/s) and launch angle (rad), then **ToF = distance / (v0 × cos(θ))**. So ToF matches the actual shot (velocity + angle) and adapts to the LUT setpoints. Two constants in `aiming.py` tune this:
  - **EXIT_VELOCITY_MPS_PER_RPS** — RPS → exit speed (m/s). Tune from one shot (e.g. 30 RPS → 12 m/s ⇒ 0.4).
  - **LAUNCH_ANGLE_RAD_PER_HOOD_ROTATION** — hood rotations → launch angle from horizontal (rad). Tune from mechanism or one known angle.
- **Optional ToF table:** `put_tof(distance_m, tof_sec)` and the `"tof"` key in `get_settings()` remain for logging or if you later add an override path; the main SOTM path uses trajectory ToF only.
- **API:** `put_rpm`, `put_hood`, `get_settings(distance_m) -> {"rpm", "hood", "tof"}` (tof in get_settings still from table seed; lead uses trajectory).

### 3. Virtual Goal Logic (SOTM)

Aiming setpoints are computed from a **virtual goal** so that when the robot is moving, we aim where the goal will be when the ball arrives.

- **Location:** `subsystems/aiming.py` — `get_aiming_parameters(...)`
- **Inputs:** Robot pose (e.g. turret center), field-relative chassis speeds, real goal pose (hub), aiming table.
- **Steps:**
  1. **Time of flight:** For **distance to real goal**, get RPS and hood from LUT; convert to exit velocity and launch angle; **ToF = distance / (v0 × cos(θ))** (trajectory). ToF = time from **ball exit to impact**.
  2. **Virtual goal:** \( G_{virtual} = G_{real} - (\vec{V}_{robot} \times \text{ToF}) \); add rotation lead: angle += omega × ToF.
  3. **Virtual distance** and **virtual angle** from robot to virtual goal
  4. **LUT lookup:** `aiming_table.get_settings(virtual_dist)` → RPS and hood for setpoints.
- **Output:** `AimingParameters(turret_angle_rad, virtual_dist_m, rps, hood_rotations)`

When the robot is stationary, field speed is zero so the virtual goal equals the real goal. Tune **EXIT_VELOCITY_MPS_PER_RPS** and **LAUNCH_ANGLE_RAD_PER_HOOD_ROTATION** in `aiming.py` so trajectory ToF matches reality (e.g. one measured shot, or compare lead vs miss).

### 4. Superstructure Coordination

- **Location:** `subsystems/superstructure.py`
- **When:** Every periodic, when goal is **LAUNCH** or **AIMHUB** and both `aim_pose_supplier` and `aiming_table` are set.
- **Behavior:**
  - Resolves real goal (hub) from alliance (e.g. `Constants.GoalLocations.BLUE_HUB` / `RED_HUB`).
  - Gets field speeds from drivetrain if present; otherwise uses `ChassisSpeeds(0, 0, 0)` (stationary).
  - Calls `get_aiming_parameters(robot_pose, field_speeds, real_goal, aiming_table)`.
  - Pushes setpoints: `turret.set_target_field_angle(...)`, `hood.set_aiming_setpoint(...)`, `launcher.set_aiming_setpoint(...)`.
- **Wiring:** `robot_container.py` builds `aim_pose_supplier` from turret center (via `make_turret_pose_supplier`) and passes `drivetrain`, `aim_pose_supplier`, and a `ShooterAimingTable()` instance into `Superstructure`.

### 5. Subsystems (Turret, Hood, Launcher)

- **Turret:** Uses the field angle from superstructure when set (`set_target_field_angle`); otherwise uses real goal (e.g. for DEPOT/OUTPOST).
- **Hood:** In AIMBOT, uses only the LUT setpoint from superstructure (`set_aiming_setpoint`); fallback if unset is `HoodConstants.STOW`.
- **Launcher:** In SCORE, uses only the LUT RPS from superstructure (`set_aiming_setpoint`); fallback if unset is the base SCORE RPS from state config.

There is **no** remaining zone-based or distance-polynomial logic in hood or launcher; all scoring aim comes from the unified aiming model.

---

## Entering Real Configuration Data From Testing

The aiming system is fully driven by `ShooterAimingTable`. Until you add tuned data, it uses the default seeded table. To use **real** configuration from testing:

### Step 1: Stationary Tuning (Golden Samples)

1. **Setup**
   - Use fresh game pieces and a charged battery (voltage compensation on if you use it).
   - Mark distances from the hub (e.g. 1 m, 2 m, 2.5 m, 3 m, 3.5 m, 4 m). Measure from **turret center** (or the same reference used by `aim_pose_supplier`).

2. **At each distance**
   - Manually adjust **launcher RPS** and **hood angle** until you get 5/5 (or your chosen repeatability) made shots.
   - Record:
     - **Distance** in **meters**.
     - **Launcher RPS** (rotations per second of the flywheel).
     - **Hood angle** in **rotations** (motor units). If you log in degrees, convert: `hood_rot = (hood_deg / 360)`.
     - **ToF** in **seconds**: time from **ball exit** (ball leaves the robot) to **impact** at the goal. Use a stopwatch, high-speed log, or dist/avg_velocity. This is used only for SOTM lead, not for when the command starts.

3. **Optional**
   - If you don’t measure ToF, the table seeds `distance/12` s; you can tune the ToF table later from moving shots or trajectory math.

### Step 2: Where to Put the Table Data

The aiming table is created in **`robot_container.py`** when constructing `Superstructure`:

```python
aiming_table=ShooterAimingTable(),
```

To use **your** tuned data:

**Option A — Build a tuned table in `robot_container.py`:**

```python
from subsystems.aiming import ShooterAimingTable

def create_tuned_aiming_table() -> ShooterAimingTable:
    table = ShooterAimingTable()
    # Replace with your (distance_m, RPS, hood_rotations, tof_sec) from testing:
    table.put_rpm(1.0, 28.0)
    table.put_hood(1.0, 0.002)
    table.put_tof(1.0, 0.12)   # ToF = time from ball exit to impact (s)
    table.put_rpm(2.0, 32.0)
    table.put_hood(2.0, 0.02)
    table.put_tof(2.0, 0.18)
    table.put_rpm(2.55, 32.0)
    table.put_hood(2.55, 0.035)
    table.put_rpm(2.9, 35.0)
    table.put_hood(2.9, 0.036)
    table.put_rpm(3.5, 38.0)
    table.put_hood(3.5, 0.04)
    table.put_rpm(4.0, 40.0)
    table.put_hood(4.0, 0.045)
    return table

# In RobotContainer.__init__, when building Superstructure:
self.superstructure = Superstructure(
    ...
    aiming_table=create_tuned_aiming_table(),
)
```

**Option B — Tuned table in a constants/config module:**

1. Add a module (e.g. `config/aiming_config.py` or in `constants.py`) that defines a function returning a `ShooterAimingTable()` with `put_rpm` / `put_hood` for every (distance_m, RPS, hood_rotations) triplet from your sheet.
2. In `robot_container.py`, import that function and pass `aiming_table=create_tuned_aiming_table()` (or similar) into `Superstructure`.

Use **Option A** for a single robot; use **Option B** if you want to share or switch configs (e.g. by robot or event).

### Step 3: Units Checklist

| Quantity      | Unit    | Notes                                      |
|---------------|---------|--------------------------------------------|
| Distance      | meters  | Same reference as `aim_pose_supplier` (e.g. turret center to hub). |
| Launcher speed| RPS     | Rotations per second (not RPM).            |
| Hood angle    | rotations | Motor units; use same units as hood sensor/encoder. |
| ToF           | seconds | Time from **ball exit** to **impact**; computed from trajectory (v0, θ). |

### Step 4: Add More Points

- Add as many (distance, RPS) and (distance, hood) points as you have from testing. The table uses **linear interpolation** between points; more points give smoother behavior.
- You can extend the table beyond your tested range; values outside the min/max distance are clamped to the nearest table value.

---

## Summary

| Component     | Role |
|--------------|------|
| **Drivetrain** | `get_field_relative_speeds()` for SOTM lead. |
| **ShooterAimingTable** | Distance → RPS, distance → hood rotations; optional ToF table. |
| **time_of_flight_trajectory()** | ToF = distance / (v0×cos(θ)); v0 and θ from LUT + constants. |
| **get_aiming_parameters()** | ToF from trajectory; virtual goal + rotation lead; returns turret angle, virtual dist, RPS, hood rotations. |
| **Superstructure** | Runs aiming when goal is LAUNCH/AIMHUB; pushes setpoints to turret, hood, launcher. |
| **Turret / Hood / Launcher** | Use only the setpoints from superstructure (with minimal fallbacks when unset). |

All aiming for scoring uses this path; there is no legacy zone or distance-polynomial logic left in the codebase.

---

## Remaining Changes for Accurate Shoot on the Move

The core SOTM pipeline (field velocity, virtual goal, LUT, hood zero) is in place. For **accurate** SOTM, do the following.

### 1. Tune trajectory ToF constants (high impact)

- **What:** ToF is computed from **velocity and angle**: ToF = distance / (v0 × cos(θ)). The conversion from LUT RPS/hood to v0 and θ uses two constants in `subsystems/aiming.py`: **EXIT_VELOCITY_MPS_PER_RPS** and **LAUNCH_ANGLE_RAD_PER_HOOD_ROTATION**. If these are wrong, the lead will be off.
- **How:** From one or two known shots (distance, RPS, hood, and either measured ToF or exit velocity), solve or tune: e.g. if 30 RPS gives ~12 m/s exit, set EXIT_VELOCITY_MPS_PER_RPS = 0.4. If one hood rotation ≈ 0.1 rad launch angle, set LAUNCH_ANGLE_RAD_PER_HOOD_ROTATION accordingly.
- **Note:** ToF is time from **ball exit to impact**, not from command start.

### 2. Rotation (omega) lead (done)

- **What:** When the robot is rotating, the turret (fixed to the robot) will have turned by `omega * ToF` by the time the ball arrives. The virtual angle now includes this: `virtual_angle += omega * tof`.
- **Status:** Implemented in `get_aiming_parameters()`.

### 3. Verify drivetrain → superstructure order

- **What:** Field velocity and pose must be from the same cycle (or very fresh). If superstructure runs before drivetrain in the same tick, it can use stale state.
- **How:** Ensure drivetrain’s `periodic()` runs before superstructure’s in `robot.py` (or that both read from the same cached state that drivetrain updates first).

### 4. Logging for tuning

- **What:** Log field velocity and ToF so you can confirm the lead in replay.
- **How:** In superstructure (when you compute aiming params), log `field_speeds.vx`, `field_speeds.vy`, `field_speeds.omega`, and ToF (e.g. `distance_to_hub / NOMINAL_BALL_VELOCITY_MPS`). Compare with observed miss direction/size when shooting on the move.

### 5. Optional: refine trajectory model

- **What:** We use ToF = distance / (v0×cos(θ)); for steep arcs you could add a gravity term (e.g. solve for t in the parabolic trajectory to reach the goal height) if needed for maximum accuracy.
