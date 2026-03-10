"""From WPILib docs, "Fire Control"."""
import math
from dataclasses import dataclass

from pykit.logger import Logger
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import radians, radians_per_second, seconds, meters


@dataclass(slots=True, frozen=True)
class AimingSample:
    """For shooting on the move, we need a 3D sample for turret aiming."""
    sample: "FiringTable.Sample"
    """Sample containing all control variables as normal."""
    turret_angle: radians
    """
    Turret target angle in radians.
    0 is facing the opponent alliance's wall, CCW positive.
    """


class FiringTable:
    """
    Essentially just https://en.wikipedia.org/wiki/Range_table

    Given a distance from the target, we interpolate through the table to find
    the needed launch angle and speed.
    """
    _MAX_ITERATIONS: int = 4
    """
    Max amount of recursions for dynamic shot calculations.
    Should normally be around 3-5. Lower if lag worsens.
    """

    _DRAG_CONSTANT: float = 0
    """
    https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
    
    Can be manually tuned for higher accuracy at high speeds.
    If 0, ignored.
    """

    @dataclass(slots=True, frozen=True)
    class Sample:
        """Stores a single entry in the firing table."""
        distance: meters
        """Distance from the hub in meters."""

        hood_angle: radians
        """Hood angle in radians, controls the launch angle."""

        flywheel_speed: radians_per_second
        """Speed of the flywheel in rad/s, controls the launch speed."""

        time_of_flight: seconds
        """Time of flight in seconds, used for recursing."""

    def __init__(self) -> None:
        """Initializes a firing table."""
        self._table: list[FiringTable.Sample] = []

    def add_sample(self, sample: Sample) -> None:
        """
        Adds a sample to the firing table,
        then sorts the list based on distance.
        """
        self._table.append(sample)
        self._table.sort(key=lambda s: s.distance)

    def get(self, distance: meters) -> Sample:
        """
        Returns a static sample
        Given the distance from the target:
        1. Find the 2 nearest samples in the firing table.
        2. Interpolate and return the new sample.

        If the desired distance is out of range, we return the highest/lowest
        sample in the table.
        """
        if len(self._table) == 0:
            raise IndexError("FiringTable does not contain any samples.")
        if distance > self._table[-1].distance:
            return self._table[-1]
        if distance < self._table[0].distance:
            return self._table[0]

        for a, b in zip(self._table, self._table[1:]):
            if a.distance <= distance <= b.distance:
                t = (distance - a.distance) / (b.distance - a.distance)
                return self._lerp(a, b, t)
        return self._table[0] # fallback

    def get_moving_shot(
        self,
        robot_pose: Pose2d,
        target_pose: Pose2d,
        robot_field_speeds: ChassisSpeeds
    ) -> AimingSample:
        """
        Returns a sample containing a target angle for shooting on the move.
        """
        # We need the target speed relative to the robot.
        # Basically if we're moving towards the target, the target's moving
        # towards us/opposite of our current speed.
        target_speeds = -robot_field_speeds

        # Get initial distance and sample
        distance = robot_pose.translation().distance(target_pose.translation())
        sample = self.get(distance)

        future_goal = target_pose.translation()
        for _ in range(self._MAX_ITERATIONS):
            # Using the time of flight from the sample,
            # we can get the distance of where the goal will be in the future.
            # Getting a new sample now gives us a new sample with a refined TOF
            # which we can repeat n times to get a refined AimingSample.
            future_goal = Translation2d(
                target_pose.X() + target_speeds.vx * sample.time_of_flight,
                target_pose.Y() + target_speeds.vy * sample.time_of_flight
            )
            distance = robot_pose.translation().distance(future_goal)
            sample = self.get(distance)

        # Now that we know where the goal will be, we can calculate the angle
        # to the goal and return an AimingSample
        angle = math.atan2(future_goal.Y() - robot_pose.Y(), future_goal.X() - robot_pose.X())
        Logger.recordOutput("FiringTable/Goal", Pose2d(future_goal.X(), future_goal.Y(), angle))
        return AimingSample(
            sample=sample,
            turret_angle=angle
        )

    @staticmethod
    def _lerp(a: Sample, b: Sample, factor: float) -> Sample:
        """There is no limit to the lerp"""
        return FiringTable.Sample(
            distance=a.distance + factor * (b.distance - a.distance),
            hood_angle=a.hood_angle + factor * (b.hood_angle - a.hood_angle),
            flywheel_speed=a.flywheel_speed + factor * (b.flywheel_speed - a.flywheel_speed),
            time_of_flight=a.time_of_flight + factor * (b.time_of_flight - a.time_of_flight)
        )

    @staticmethod
    def _effective_time_of_flight(time_of_flight: float) -> float:
        if FiringTable._DRAG_CONSTANT == 0:
            return time_of_flight
        return (
                (1 - math.exp(
                    -FiringTable._DRAG_CONSTANT * time_of_flight
                ))
                / FiringTable._DRAG_CONSTANT
        )
