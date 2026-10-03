"""Logic abstraction for Feeder IO layers"""
from enum import IntEnum, auto
from typing import Final

from pykit.logger import Logger
from wpilib import Alert

from subsystems import StateSubsystem
from subsystems.feeder.io import FeederIO, FeederIOTalonFX, FeederIOSim

__all__ = ["FeederIO", "FeederIOTalonFX", "FeederIOSim", "FeederSubsystem"]


class FeederSubsystem(StateSubsystem):
    """
    The FeederSubsystem is responsible for storage and feeding game pieces
    into the launcher.
    """

    class SubsystemState(IntEnum):
        """Available subsystem states"""
        STOP = auto()
        INWARD = auto()

    _state_configs: dict[SubsystemState, float] = {
        SubsystemState.STOP: 0.0,
        SubsystemState.INWARD: 80.0, # TODO:was 30 for week 1, and we need to test if this fixes stuck fuel
    }

    def __init__(self, io: FeederIO) -> None:
        super().__init__("Feeder", self.SubsystemState.STOP)

        self._io: Final[FeederIO] = io
        self._inputs = FeederIO.FeederIOInputs()

        # Alert for disconnected motor
        self._motor_disconnected_alert = Alert(
            "Feeder motor is disconnected.",
            Alert.AlertType.kError
        )

    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.update_inputs(self._inputs)

        # Log inputs to PyKit
        Logger.processInputs("Feeder", self._inputs)

        # Update alerts
        self._motor_disconnected_alert.set(not self._inputs.motor_connected)

        super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        # Get motor rps for this state
        motor_rps = self._state_configs.get(
            desired_state,
            0.0
        )

        # Set motor velocity through IO layer
        self._io.set_motor_rps(motor_rps)
