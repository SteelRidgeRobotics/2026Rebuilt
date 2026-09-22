"""Logic abstraction for Turret IO layers"""
from enum import IntEnum, auto
from math import atan2
from typing import Final, Callable, Optional

from pykit.logger import Logger
from wpilib import Alert, DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Pose3d, Rotation3d

from constants import Constants
from subsystems import StateSubsystem
from subsystems.turret.io import TurretIO


#pylint: disable=too-many-instance-attributes
class TurretSubsystem(StateSubsystem):
    """
    Shooter yaw is mechanically fixed to the chassis.

    This subsystem still tracks which field goal to face (hub / depot /
    outpost) and holds the turret motor at home. Chassis heading does the
    aiming.
    """

    class SubsystemState(IntEnum):
        """All turret states"""
        MANUAL = auto()
        HUB = auto()
        DEPOT = auto()
        OUTPOST = auto()

    _FIXED_POSITION_RAD: Final[float] = 0.0

    def __init__(self,
                 io: TurretIO,
                 robot_pose_supplier: Callable[[], Pose2d]
                 ) -> None:
        super().__init__("Turret", self.SubsystemState.MANUAL)

        self._io: Final[TurretIO] = io
        self.inputs = TurretIO.TurretIOInputs()
        self.set_desired_state(TurretSubsystem.SubsystemState.MANUAL)
        self.robot_pose_supplier = robot_pose_supplier

        self.turret_disconnected_alert = Alert(
            "Turret motor is disconnected.",
            Alert.AlertType.kError
        )

        self.target_radians = 0.0
        self._target_field_angle: Optional[
            float] = None  # SOTM virtual goal angle (rad), None = use real
        # goal

        self.x = 6.7
        self.y = 4.1

    def set_target_field_angle(self, angle_rad: Optional[float]) -> None:
        """Set field-frame aim angle (rad). When None, uses the real goal."""
        self._target_field_angle = angle_rad

    def periodic(self):

        # Update inputs from hardware/simulation
        self._io.update_inputs(self.inputs)

        # Log inputs to PyKit
        Logger.processInputs("Turret", self.inputs)
        Logger.recordOutput("Turret/Target Radians", self.target_radians)

        # Update alerts
        self.turret_disconnected_alert.set(not self.inputs.turret_connected)

        # Mechanism is fixed; keep the motor at home.
        self._io.set_position(self._FIXED_POSITION_RAD)

        if self.get_current_state() != self.SubsystemState.MANUAL:
            heading = self.get_aim_field_heading()
            self.target_radians = heading.radians()

        super().periodic()

    def get_radians_to_goal(self) -> float:
        """
        Field-frame angle (position) from robot to goal. 0 = +X (red alliance
        wall), CCW positive.
        Returns 0 for MANUAL or if robot is at goal.
        """
        state = self.get_current_state()
        if state == self.SubsystemState.MANUAL:
            return 0.0

        robot = self.robot_pose_supplier().translation()
        goal = self._goal_pose_for_state(state).translation()

        dx = goal.X() - robot.X()
        dy = goal.Y() - robot.Y()
        self.x = abs(dx)
        self.y = abs(dy)

        if dx == 0.0 and dy == 0.0:
            return 0.0
        return atan2(dy, dx)

    def get_aim_field_heading(self) -> Rotation2d:
        """Robot heading that points the fixed shooter at the current goal."""
        if self._target_field_angle is not None:
            return Rotation2d(self._target_field_angle)
        return Rotation2d(self.get_radians_to_goal())

    def _goal_pose_for_state(self, state: SubsystemState) -> Pose2d:
        """Goal pose for the given state and current alliance."""
        is_blue = DriverStation.getAlliance() == DriverStation.Alliance.kBlue
        match state:
            case self.SubsystemState.HUB:
                return Constants.GoalLocations.BLUE_HUB if is_blue else (
                    Constants.GoalLocations.RED_HUB)
            case self.SubsystemState.OUTPOST:
                return Constants.GoalLocations.BLUE_OUTPOST_PASS if is_blue \
                    else Constants.GoalLocations.RED_OUTPOST_PASS
            case self.SubsystemState.DEPOT:
                return Constants.GoalLocations.BLUE_DEPOT_PASS if is_blue \
                    else Constants.GoalLocations.RED_DEPOT_PASS
            case _:
                return Constants.GoalLocations.BLUE_HUB  # fallback, caller
                # should not use for MANUAL

    def get_current_state(self) -> SubsystemState | None:
        """get state"""
        return super().get_current_state()

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        """set state"""
        if not super().set_desired_state(desired_state):
            return
        self._io.set_position(self._FIXED_POSITION_RAD)

    def get_component_pose(self) -> Pose3d:
        """Gets the articulated component pose for AdvantageScope."""
        return Pose3d(
            -0.1524,
            0,
            0,
            Rotation3d(0, 0, 0)
        )
