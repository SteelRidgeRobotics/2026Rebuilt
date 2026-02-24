from abc import ABC
from dataclasses import dataclass
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from pykit.autolog import autolog
from wpilib.simulation import DCMotorSim

from wpimath.units import radians, radians_per_second, amperes, celsius
from wpimath.trajectory import TrapezoidProfile
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.controller import PIDController
from math import pi

from constants import Constants
from util import tryUntilOk


class FeederIO(ABC):
    """
    Abstract base class for Feeder IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass
    class FeederIOInputs:
        """Inputs from the Feeder hardware/simulation."""
        # Motor status
        motorConnected: bool = False
        motorPosition: radians = 0.0
        motorVelocity: radians_per_second = 0.0
        motorAppliedVolts: float = 0.0
        motorCurrent: amperes = 0.0
        motorTemperature: celsius = 0.0


    def updateInputs(self, inputs: FeederIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""
        pass

    def setMotorRPS(self, rps: float) -> None:
        """Set the motor output rotations per second."""
        pass


class FeederIOTalonFX(FeederIO):
    """
    Real hardware implementation using TalonFX motor controller.
    """

    """
    Real hardware implementation using TalonFX motor controller.
    """

    def __init__(self) -> None:
        """
        Initialize the real hardware IO.
        """
        self._motor: Final[TalonFX] = TalonFX(Constants.CanIDs.FEEDER_TALON, "rio")

        # Apply motor configuration
        _motor_config = TalonFXConfiguration()

        _motor_config.slot0 = Constants.FeederConstants.GAINS
        _motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        _motor_config.feedback.sensor_to_mechanism_ratio = Constants.FeederConstants.GEAR_RATIO

        tryUntilOk(5, lambda: self._motor.configurator.apply(_motor_config, 0.25))
        tryUntilOk(5, lambda: self._motor.set_position(0, 0.25))

        # Create status signals for motor
        self._position: Final = self._motor.get_position()
        self._velocity: Final = self._motor.get_velocity()
        self._appliedVolts: Final = self._motor.get_motor_voltage()
        self._current: Final = self._motor.get_stator_current()
        self._temperature: Final = self._motor.get_device_temp()

        # Configure update frequencies
        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )
        self._motor.optimize_bus_utilization()

        # Voltage control request
        self._velocityRequest: Final[VelocityVoltage] = VelocityVoltage(0)
        self._velocityRequest.feed_forward = Constants.FeederConstants.FEED_FORWARD
        self._voltageRequest: Final[VoltageOut] = VoltageOut(0)

    def updateInputs(self, inputs: FeederIO.FeederIOInputs) -> None:
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
        """Set the motor output voltage."""
        if rps == 0:
            self._motor.set_control(self._voltageRequest)
        else:
            self._velocityRequest.velocity = rps
            self._velocityRequest.feed_forward = Constants.FeederConstants.FEED_FORWARD
            self._motor.set_control(self._velocityRequest)


class FeederIOSim(FeederIO):
    """
    Simulation implementation for testing without hardware.
    """

    def __init__(self) -> None:
        """Initialize the simulation IO."""
        self._motorType = DCMotor.krakenX60(1)

        linearSystem = LinearSystemId.DCMotorSystem(
            self._motorType,
            Constants.FeederConstants.MOMENT_OF_INERTIA,
            Constants.FeederConstants.GEAR_RATIO
        )
        self._simMotor = DCMotorSim(linearSystem, self._motorType, [0, 0])
        self._closedLoop = True

        self._motorPosition: float = 0.0
        self._motorVelocity: float = 0.0
        self._motorAppliedVolts: float = 0.0

        self._controller = PIDController(
                            Constants.FeederConstants.GAINS.k_p / (2*pi),
                            Constants.FeederConstants.GAINS.k_i / (2*pi),
                            Constants.FeederConstants.GAINS.k_d / (2*pi),
                            0.02)

    def updateInputs(self, inputs: FeederIO.FeederIOInputs) -> None:
        """Update inputs with simulated state."""
        
        self._simMotor.update(0.02)

        if self._closedLoop:
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
        """Set the motor output RPS (revolutions per second) (simulated)."""
        self._controller.setSetpoint(rps)
