"""
Hood subsystem

TO DO:
Integrate positions for passing (may switch to state subsystem for this)
"""
from enum import IntEnum, auto
from typing import Callable, Optional

from pathplannerlib.auto import FlippingUtil, AutoBuilder
from pykit.logger import Logger
from wpilib import Alert, DriverStation
from wpimath.geometry import Pose2d, Pose3d, Rotation3d
from wpimath.units import rotationsToRadians

from constants import Constants
from subsystems import StateSubsystem
from subsystems.hood.io import HoodIO


# pylint: disable=too-many-instance-attributes
class HoodSubsystem(StateSubsystem):
    """State Subsystem for hood."""

    class SubsystemState(IntEnum):
        """Hood states."""
        AIMBOT = auto()
        STOW = auto()
        PASS = auto()
        MANUAL = auto()

    _state_configs: dict[SubsystemState, tuple[bool, float]] = {
        SubsystemState.AIMBOT: (True, 0.0),
        SubsystemState.STOW: (False, Constants.HoodConstants.STOW),
        SubsystemState.PASS: (False, Constants.HoodConstants.PASSING),
        SubsystemState.MANUAL: (False, 0.0)
    }

    def __init__(self,
                 hood_io: HoodIO,
                 robot_pose_supplier: Callable[[], Pose2d]
                 ) -> None:
        super().__init__("Hood", self.SubsystemState.STOW)

        self.io = hood_io
        self.alliance = DriverStation.getAlliance()
        self.set_desired_state(HoodSubsystem.SubsystemState.STOW)

        self.robot_pose_supplier = robot_pose_supplier

        self.inputs = HoodIO.HoodIOInputs()
        self.hood_disconnected_alert = Alert(
            "Hood motor is disconnected.",
            Alert.AlertType.kError
        )

        self.hub_pose = Constants.FieldConstants.HUB_POSE  # blue hub
        self.distance = 1.0
        self.target = 1.0
        self._aiming_hood_setpoint: Optional[
            float] = None  # From unified aiming LUT (rotations)

    def set_aiming_setpoint(self, hood_rotations: Optional[float]) -> None:
        """Set hood angle from unified aiming LUT (rotations, motor units)."""
        self._aiming_hood_setpoint = hood_rotations

    def periodic(self) -> None:
        """Runs stuff periodically (every 20 ms)."""
        self.alliance = DriverStation.getAlliance()
        self.hub_pose = Constants.FieldConstants.HUB_POSE if not (
            AutoBuilder.shouldFlip()) else FlippingUtil.flipFieldPose(
            Constants.FieldConstants.HUB_POSE
        )

        self.io.update_inputs(self.inputs)

        if self._auto_aim:
            self.distance = (self.robot_pose_supplier()
            .translation().distance(
                self.hub_pose.translation()
            ))
            if self._aiming_hood_setpoint is not None:
                # LUT value is rotations from zero (hood down); add zero
                # position for absolute motor setpoint
                self.target = self._aiming_hood_setpoint
            else:
                self.target = Constants.HoodConstants.STOW
            self.io.set_position(self.target)

        self.hood_disconnected_alert.set(not self.inputs.hood_connected)

        Logger.processInputs("Hood", self.inputs)
        Logger.recordOutput("Hood/Distance", self.distance)
        Logger.recordOutput("Hood/Target", self.target)

        super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        super().set_desired_state(desired_state)

        self._auto_aim, hood_pos = self._state_configs.get(desired_state, 0.0)
        self.io.set_position(hood_pos)

    def get_current_state(self) -> SubsystemState | None:
        """get state"""
        return super().get_current_state()