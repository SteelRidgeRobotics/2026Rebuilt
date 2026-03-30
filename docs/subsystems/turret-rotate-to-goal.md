# Turret `rotate_to_goal` — Synopsis and Formulas

## Synopsis

**Purpose:** Drives the turret to the goal angle in turret (motor) space while:

1. Using the **virtual goal** angle when SOTM is active, otherwise the real goal.
2. Converting **field angle → robot-relative turret angle** and wrapping to [−π, π].
3. Mapping that angle into the **physical range** [θ_min, θ_max] (from `MIN_ROTATIONS` / `MAX_ROTATIONS`) with wrap and clamp.
4. Applying **hysteresis at center** so the turret only crosses the middle when the goal is at least 5° past center, avoiding flip-flop across the hard stop.

If the turret is in `MANUAL` state, the function returns without commanding.

---

## Formulas

### 1. Field angle source

θ_field = θ_target if θ_target is set (SOTM virtual goal), else θ_goal (real goal from `get_radians_to_goal()`).

### 2. Turret angle relative to robot (motor convention)

Field is CCW-positive; turret motor is CW-positive, so:

**θ_turret = −(θ_field − θ_robot)**

with θ_robot from the pose supplier. Then θ_turret is wrapped into [−π, π] by adding/subtracting 2π as needed.

### 3. Physical range

**θ_min = 2π · MIN_ROTATIONS**  
**θ_max = 2π · MAX_ROTATIONS**

θ_turret is mapped into [θ_min, θ_max] by adding/subtracting 2π until it lies in that interval, then clamped:

**θ_in_range = clamp(θ_turret mod [θ_min, θ_max], θ_min, θ_max)**

(Code does the “mod” by repeated ±2π and then clamps.)

### 4. Center and hysteresis

**θ_middle = (θ_min + θ_max) / 2**  
**θ_hyst = 5π / 180**

### 5. Hysteresis rule (final commanded angle)

- **θ_cmd = θ_current** if on left and goal on right and θ_in_range < θ_middle + θ_hyst
- **θ_cmd = θ_current** if on right and goal on left and θ_in_range > θ_middle − θ_hyst
- **θ_cmd = θ_in_range** otherwise

- **On left:** θ_current < θ_middle
- **On right:** θ_current ≥ θ_middle
- Goal side is determined by θ_in_range vs θ_middle.

The function then sets the turret position to θ_cmd via `_io.set_position(command_turret)` and stores θ_field in `target_radians`.

---

## `get_radians_to_goal`

**Purpose:** Returns the field-frame angle from the robot to the goal, used as the real-goal angle (θ_goal) when SOTM virtual goal is not set.

**Convention:** 0 = +X (red alliance wall), counter-clockwise positive.

**Returns 0** when the turret state is MANUAL or when the robot is exactly at the goal (dx = 0 and dy = 0).

**Formula:**

- Robot position from pose supplier: (robot.X(), robot.Y()).
- Goal position from `_goal_pose_for_state(state)`: (goal.X(), goal.Y()).
- **dx = goal.X() − robot.X()**
- **dy = goal.Y() − robot.Y()**
- **θ_goal = atan2(dy, dx)**

So the returned angle is the bearing from the robot to the goal in the field coordinate frame.
