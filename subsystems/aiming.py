"""
Shooting on the Move (SOTM) and Unified Kinematic Aiming.

Provides ShooterAimingTable (distance -> RPM/hood LUT) and
get_aiming_parameters()
for Virtual Goal logic so turret, hood, and launcher can aim and shoot while
moving.

ToF (time from ball exit to impact) is computed from trajectory: exit velocity
and launch angle from the LUT, then ToF = distance / (v0 * cos(theta)).
Override with put_tof() for a table-based ToF if preferred.
"""
import math
from dataclasses import dataclass

from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from constants import Constants

# Trajectory ToF: convert LUT RPS and hood to v0 (m/s) and launch angle (rad).
# Tune from one known shot (e.g. 30 RPS -> ~12 m/s => 0.4 m/s per RPS).
EXIT_VELOCITY_MPS_PER_RPS = 0.4
# Hood rotations (motor) to launch angle from horizontal (rad). Tune from mechanism.
LAUNCH_ANGLE_RAD_PER_HOOD_ROTATION = 2.0 * math.pi * 0.15  # ~0.94 rad per motor rot
GRAVITY_MPS2 = 9.80665


def time_of_flight_trajectory(
    distance_m: float,
    exit_velocity_mps: float,
    launch_angle_rad: float,
) -> float:
    """
    Time from ball exit to impact (seconds) from 2D trajectory.
    Uses horizontal range: ToF = distance / (v0 * cos(theta)).
    Clamps cos(theta) to avoid div by zero; ignores goal height for simplicity.
    """
    if exit_velocity_mps <= 0.0:
        return distance_m / 12.0  # fallback
    cos_theta = math.cos(launch_angle_rad)
    cos_theta = max(min(cos_theta, 1.0), 0.01)  # clamp for stability
    return distance_m / (exit_velocity_mps * cos_theta)


def _linear_interp(x: float, xs: list[float], ys: list[float]) -> float:
    """Linear interpolation. Clamps to [min(ys), max(ys)] if x outside [min(
    xs), max(xs)]."""
    if not xs or not ys or len(xs) != len(ys):
        return ys[0] if ys else 0.0
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1]:
            t = (x - xs[i]) / (xs[i + 1] - xs[i]) if xs[i + 1] != xs[
                i] else 0.0
            return ys[i] + t * (ys[i + 1] - ys[i])
    return ys[-1]


class ShooterAimingTable:
    """
    Distance-based LUT for RPM, hood angle, and time-of-flight (continuous
    interpolation). Populate with golden samples from stationary tuning.
    ToF = time from ball exit to impact (seconds).
    """

    def __init__(self) -> None:
        # (distance_m, value) sorted by distance. Hood in rotations (motor
        # units). ToF in seconds.
        self._rpm_dist: list[float] = []
        self._rpm_val: list[float] = []
        self._hood_dist: list[float] = []
        self._hood_val: list[float] = []
        self._tof_dist: list[float] = []
        self._tof_val: list[float] = []
        self._seed_defaults()

    def _seed_defaults(self) -> None:
        """Seed with tuned values from stationary testing (distance m,
        hood rotations, flywheel RPS, ToF sec)."""
        # From testing: Distance (m), Hood (rotations), Flywheel (RPS), ToF (s)
        distance = [1.629, 1.744, 1.996, 2.504, 2.55, 3.011, 3.502, 4.293, 5.043, 5.879, 8, 10]
        hood_rotations = [0, 0.0205, 0, 0.021, 0.02, 0.03, 0.03, 0.039, 0.041, 0.045, 0.054, 0.054]  # from zero (hood down = 0)
        flywheel_rps = [25,27,28,29,30,30,32,41,46,54,80,90]
        for i in range(len(flywheel_rps)):
            #flywheel_rps[i] += Constants.LauncherConstants.LAUNCH_OFFSET
            flywheel_rps[i] *= Constants.LauncherConstants.LAUNCH_OFFSET
            if flywheel_rps[i] >= 90:
                flywheel_rps[i] = 90
        # ToF = time from ball exit to impact (s); seed with dist/12 m/s
        tof_sec = [d / 12.0 for d in distance]
        self._rpm_dist = list(distance)
        self._rpm_val = list(flywheel_rps)
        self._hood_dist = list(distance)
        self._hood_val = list(hood_rotations)
        self._tof_dist = list(distance)
        self._tof_val = list(tof_sec)

    def put_rpm(self, distance_m: float, rpm: float) -> None:
        """Add or update one RPM sample (distance in meters)."""
        self._add_sample(distance_m, rpm, self._rpm_dist, self._rpm_val)

    def put_hood(self, distance_m: float, hood_rotations: float) -> None:
        """Add or update one hood angle sample (distance in meters, hood in
        rotations)."""
        self._add_sample(
            distance_m,
            hood_rotations,
            self._hood_dist,
            self._hood_val
        )

    def put_tof(self, distance_m: float, tof_sec: float) -> None:
        """Add or update one ToF sample (distance in meters, time in seconds).
        ToF = time from ball exit to impact."""
        self._add_sample(distance_m, tof_sec, self._tof_dist, self._tof_val)

    @staticmethod
    def _add_sample(
        x: float,
        y: float,
        xs: list[float],
        ys: list[float]
    ) -> None:
        if not xs or x < xs[0]:
            xs.insert(0, x)
            ys.insert(0, y)
            return
        if x > xs[-1]:
            xs.append(x)
            ys.append(y)
            return
        for i, xi in enumerate(xs):
            if abs(xi - x) < 1e-9:
                ys[i] = y
                return
            if xi > x:
                xs.insert(i, x)
                ys.insert(i, y)
                return

    def get_settings(self, distance_m: float) -> dict[str, float]:
        """Returns {'rpm': RPS, 'hood': rotations, 'tof': seconds} for the
        given distance. tof = time from ball exit to impact."""
        return {
            "rpm": _linear_interp(distance_m, self._rpm_dist, self._rpm_val),
            "hood": _linear_interp(
                distance_m,
                self._hood_dist,
                self._hood_val
            ),
            "tof": _linear_interp(distance_m, self._tof_dist, self._tof_val),
        }


@dataclass
class AimingParameters:
    """Result of Virtual Goal aiming: setpoints for turret, hood,
    and launcher."""
    turret_angle_rad: float
    virtual_dist_m: float
    rps: float  # launcher wheel RPS (from LUT, same units as
    # LauncherSubsystem.desired_motor_rps)
    hood_rotations: float  # hood angle in rotations (motor units)


# pylint: disable=too-many-locals
def get_aiming_parameters(
    robot_pose: Pose2d,
    field_speeds: ChassisSpeeds,
    real_goal_pose: Pose2d,
    aiming_table: ShooterAimingTable,
) -> AimingParameters:
    """
    Virtual Goal (SOTM) logic: aim at where the goal will appear when the
    ball arrives.

    ToF = time from **ball exit** to **impact**. Computed from trajectory:
    get RPS and hood for this distance from LUT, convert to exit velocity and
    launch angle, then ToF = distance / (v0 * cos(theta)).
    """
    rx = robot_pose.X()
    ry = robot_pose.Y()
    gx = real_goal_pose.X()
    gy = real_goal_pose.Y()
    dist_to_real = max(math.hypot(gx - rx, gy - ry), 1e-6)
    # ToF from trajectory (velocity + angle) for the shot at this distance
    settings_for_dist = aiming_table.get_settings(dist_to_real)
    v0 = settings_for_dist["rps"] * EXIT_VELOCITY_MPS_PER_RPS
    launch_angle_rad = settings_for_dist["hood"] * LAUNCH_ANGLE_RAD_PER_HOOD_ROTATION
    tof = time_of_flight_trajectory(dist_to_real, v0, launch_angle_rad)
    # Virtual goal (look-ahead)
    vgx = gx - (field_speeds.vx * tof)
    vgy = gy - (field_speeds.vy * tof)
    virtual_dist = math.hypot(vgx - rx, vgy - ry)
    virtual_angle = math.atan2(vgy - ry, vgx - rx)
    # Lead robot rotation: by ToF the robot (and turret) will have rotated by omega*tof
    virtual_angle += field_speeds.omega * tof
    settings = aiming_table.get_settings(virtual_dist)
    return AimingParameters(
        turret_angle_rad=virtual_angle,
        virtual_dist_m=virtual_dist,
        rps=settings["rpm"],
        hood_rotations=settings["hood"],
    )
