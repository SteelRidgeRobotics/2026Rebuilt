import importlib.util
import sys
from pathlib import Path

from subsystems.feeder.io import FeederIO, FeederIOTalonFX, FeederIOSim
from enum import auto, Enum

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert
from typing import Final
from constants import Constants
from subsystems import StateSubsystem

__all__ = ["FeederIO", "FeederIOTalonFX", "FeederIOSim", "FeederSubsystem"]

class FeederSubsystem(StateSubsystem):
    """
    The FeederSubsystem is responsible for storage and feeding game pieces into the launcher.
    """

    class SubsystemState(Enum):
        STOP = auto()
        INWARD = auto()

    _state_configs: dict[SubsystemState, float] = {
        SubsystemState.STOP: 0.0,
        SubsystemState.INWARD: 40.0,
    }

    def __init__(self, io: FeederIO) -> None:
        super().__init__("Feeder", self.SubsystemState.STOP)

        self._io: Final[FeederIO] = io
        self._inputs = FeederIO.FeederIOInputs()
        
        # Alert for disconnected motor
        self._motorDisconnectedAlert = Alert("Feeder motor is disconnected.", Alert.AlertType.kError)

    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)
        
        # Log inputs to PyKit
        Logger.processInputs("Feeder", self._inputs)
        
        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        # Get motor rps for this state
        motor_rps = self._state_configs.get(
            desired_state, 
            0.0
        )
        
        # Set motor velocity through IO layer
        self._io.setMotorRPS(motor_rps)

    