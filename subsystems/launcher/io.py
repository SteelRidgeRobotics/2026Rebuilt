from abc import ABC
from dataclasses import dataclass, field
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage, Follower, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, MotorAlignmentValue, InvertedValue

from pykit.autolog import autolog
from wpimath.units import radians, radians_per_second, volts, amperes, celsius
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.controller import PIDController
from wpilib.simulation import FlywheelSim
from math import pi

from constants import Constants
LauncherConstants = Constants.LauncherConstants
GeneralConstants = Constants.GeneralConstants
CanIds = Constants.CanIDs

from util import tryUntilOk


class LauncherIO(ABC):
    """
    Abstract base class for Launcher IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass
    class LauncherIOInputs:
        """Inputs from the Launcher hardware/simulation."""
        # Motor status
        motorConnected: bool = False
        motorPosition: radians = 0.0
        motorVelocity: radians_per_second = 0.0
        motorAppliedVolts: volts = 0.0
        motorCurrent: amperes = 0.0
        motorTemperature: celsius = 0.0


    def updateInputs(self, inputs: LauncherIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""
        pass

    def setMotorRPS(self, rps: float) -> None:
        """Set the motor output velocity."""
        pass


class LauncherIOTalonFX(LauncherIO):
    """
    Real hardware implementation using TalonFX motor controller.
    """

    def __init__(self) -> None:
        """
        Initialize the real hardware IO.
        """
        self._main_motor: Final[TalonFX] = TalonFX(CanIds.LAUNCHER_TOP_TALON, "rio")
        self._follower_motor: Final[TalonFX] = TalonFX(CanIds.LAUNCHER_LOW_TALON, "rio")

        # Apply motor configuration
        _motor_config = TalonFXConfiguration()

        _motor_config.slot0 = LauncherConstants.GAINS
        _motor_config.motor_output.neutral_mode = NeutralModeValue.COAST
        _motor_config.feedback.sensor_to_mechanism_ratio = LauncherConstants.GEAR_RATIO
        _motor_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        tryUntilOk(5, lambda: self._main_motor.configurator.apply(_motor_config, 0.25))

        # Create status signals for motor
        self._position: Final = self._main_motor.get_position()
        self._velocity: Final = self._main_motor.get_velocity()
        self._appliedVolts: Final = self._main_motor.get_motor_voltage()
        self._current: Final = self._main_motor.get_stator_current()
        self._temperature: Final = self._main_motor.get_device_temp()

        # Configure update frequencies
        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )
        self._main_motor.optimize_bus_utilization()

        # Control requests
        self._velocityRequest: Final[VelocityVoltage] = VelocityVoltage(0)
        self._voltageRequest: Final[VoltageOut] = VoltageOut(0)
        self._follower_motor.set_control(Follower(CanIds.LAUNCHER_TOP_TALON, MotorAlignmentValue.ALIGNED))

    def updateInputs(self, inputs: LauncherIO.LauncherIOInputs) -> None:
        """Update inputs with current motor state."""
        # Refresh all motor signals
        motorStatus = BaseStatusSignal.refresh_all(
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )

        # Update motor inputs
        inputs.motorConnected = motorStatus.is_ok()
        inputs.motorPosition = self._position.value_as_double
        inputs.motorVelocity = self._velocity.value_as_double
        inputs.motorAppliedVolts = self._appliedVolts.value_as_double
        inputs.motorCurrent = self._current.value_as_double
        inputs.motorTemperature = self._temperature.value_as_double

    def setMotorRPS(self, rps: float) -> None:
        """Set the motor output velocity."""
        if rps == 0:
            self._voltageRequest = VoltageOut(0)
            self._main_motor.set_control(self._voltageRequest)
        else:
            self._velocityRequest.velocity = rps
            self._main_motor.set_control(self._velocityRequest)


class LauncherIOSim(LauncherIO):
    """
    Simulation implementation for testing without hardware.
    """

    def __init__(self) -> None:
        """Initialize the simulation IO."""
        self._motorType = DCMotor.krakenX44FOC(2) 

        linearSystem = LinearSystemId.flywheelSystem(
            self._motorType,
            LauncherConstants.MOMENT_OF_INERTIA,
            LauncherConstants.GEAR_RATIO
        )
        self._simMotor = FlywheelSim(linearSystem, self._motorType, [0])
        self._closedloop = True

        self._motorPosition: float = 0.0
        self._motorVelocity: float = 0.0
        self._motorAppliedVolts: float = 0.0

        self._controller = PIDController(
                            LauncherConstants.GAINS.k_p / (2*pi),
                            LauncherConstants.GAINS.k_i / (2*pi),
                            LauncherConstants.GAINS.k_d / (2*pi),
                            0.02)
                        
    def updateInputs(self, inputs: LauncherIO.LauncherIOInputs) -> None:
        """Update inputs with simulated state."""

        self._simMotor.update(0.02)

        if self._closedloop:
            self._motorAppliedVolts = self._controller.calculate(
                self._simMotor.getAngularVelocity())
        else:
            self._controller.reset()

        self._simMotor.setInputVoltage(self._motorAppliedVolts)

        # Update inputs
        inputs.motorConnected = True
        inputs.motorVelocity = self._simMotor.getAngularVelocity()
        inputs.motorAppliedVolts = self._simMotor.getInputVoltage()
        inputs.motorCurrent = self._simMotor.getCurrentDraw()
        inputs.motorTemperature = 25.0
        inputs.motorPosition += inputs.motorVelocity * 0.02


    def setMotorRPS(self, rps: float) -> None:
        """Set the motor output velocity."""
        self._controller.setSetpoint(rps)