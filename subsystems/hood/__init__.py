"""
Hood subsystem

TO DO:
Integrate positions for passing (may switch to state subsystem for this)
"""
from enum import Enum, auto
from math import atan, sqrt, degrees
from typing import Callable

from pathplannerlib.auto import FlippingUtil, AutoBuilder
from pykit.logger import Logger
from wpilib import Alert, DriverStation
from wpimath.geometry import Pose2d, Pose3d
from wpimath.units import degreesToRotations

from constants import Constants
from subsystems import StateSubsystem
from subsystems.hood.io import HoodIO


# pylint: disable=too-many-instance-attributes
class HoodSubsystem(StateSubsystem):
    """State Subsystem for hood."""

    class SubsystemState(Enum):
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

    def __init__(self, io: HoodIO, robot_pose_supplier: Callable[[], Pose2d]) -> None:
        super().__init__("Hood", self.SubsystemState.STOW)

        self.io = io
        self.alliance = DriverStation.getAlliance()
        self.set_desired_state(HoodSubsystem.SubsystemState.STOW)

        self.robot_pose_supplier = robot_pose_supplier

        self.inputs = HoodIO.HoodIOInputs()
        self.hood_disconnected_alert = Alert("Hood motor is disconnected.", Alert.AlertType.kError)

        #self.hub_pose = Constants.FieldConstants.HUB_POSE  # blue hub
        self.distance = 1.0
        self.target = 1.0

    def interpolate(self) -> None:
        """Updates hood angle."""
        if self.distance <= Constants.HoodConstants.MAX_DISTANCE_FOR_SLOW_LAUNCH:
            self.target = 0.0000840628 * (self.distance ** 6.40933)
        elif self.distance <= Constants.HoodConstants.MAX_DISTANCE_FOR_MEDIUM_LAUNCH and self.distance > Constants.HoodConstants.MAX_DISTANCE_FOR_SLOW_LAUNCH:
            self.target = 0.03637695313 # Using a hardcoded value for this small range
        else:
            self.target = 0.0000139591 * (self.distance ** 5.1281) 

    def periodic(self) -> None:
        """Runs stuff periodically (every 20 ms)."""
        self.alliance = DriverStation.getAlliance()
        self.hub_pose = Constants.FieldConstants.HUB_POSE if not (
            AutoBuilder.shouldFlip()) else FlippingUtil.flipFieldPose(Constants.FieldConstants.HUB_POSE)
        
        self.io.update_inputs(self.inputs)

        if self._auto_aim:
            self.distance = (self.robot_pose_supplier()
                         .translation().distance(self.hub_pose.translation()))
            self.interpolate()
            self.io.set_position(self.target)


        self.hood_disconnected_alert.set(not self.inputs.hood_connected)

        Logger.processInputs("Hood", self.inputs)
        Logger.recordOutput("Hood/Distance", self.distance)
        Logger.recordOutput("Hood/Target", self.target)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        """set state"""
        if not super().set_desired_state(desired_state):
            return

        self._auto_aim, hood_pos = self._state_configs.get(desired_state, 0.0)
        self.io.set_position(hood_pos)

    def get_current_state(self) -> SubsystemState | None:
        """get state"""
        return super().get_current_state()

    def get_component_pose(self) -> Pose3d:
        """For advantage scope modelling (placeholder)."""

    def rotate_manually(self, axis: float): # Axis is the value of the X-axis from a joystick
        self.set_desired_state(self.SubsystemState.MANUAL)
        target_velocity = axis * Constants.HoodConstants.MAX_MANUAL_VELOCITY
        self.io.set_velocity(target_velocity)