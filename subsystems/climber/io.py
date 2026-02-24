from abc import ABC
from dataclasses import dataclass
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFX
from pykit.autolog import autolog
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radians, radians_per_second, volts, amperes, celsius, radiansToRotations
from wpimath.controller import PIDController
from math import pi

from constants import Constants
from util import tryUntilOk


class ClimberIO(ABC):
    """
    Abstract base class for climber IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass
    class ClimberIOInputs:
        """Inputs from the climber hardware/simulation."""
        # Motor status
        climber_connected: bool = False
        climber_position: radians = 0.0
        climber_velocity: radians_per_second = 0.0
        climber_applied_volts: volts = 0.0
        climber_current: amperes = 0.0
        climber_temperature: celsius = 0.0
        climber_zero_position: float = 0.0

    def update_inputs(self, inputs: ClimberIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""
        pass

    def set_open_loop(self, output: float) -> None:
        pass

    def set_position(self, radians: float) -> None:
        """Set the climber position"""
        pass

class ClimberIOTalonFX(ClimberIO):
    """
    Real hardware implementation using TalonFX motor controller and Servo.
    """

    def __init__(self, motor_id: int, motor_config: TalonFXConfiguration) -> None:
        """
        Initialize the real hardware IO.

        :param motor_id: CAN ID of the TalonFX motor
        :param motor_config: TalonFX configuration to apply
        """
        self._motor: Final[TalonFX] = TalonFX(motor_id, "*")

        # Apply motor configuration
        tryUntilOk(5, lambda: self._motor.configurator.apply(motor_config, 0.25))
        tryUntilOk(5, lambda: self._motor.set_position(0, 0.25))

        # Create status signals for motor
        self._position: Final = self._motor.get_position()
        self._velocity: Final = self._motor.get_velocity()
        self._applied_volts: Final = self._motor.get_motor_voltage()
        self._current: Final = self._motor.get_stator_current()
        self._temperature: Final = self._motor.get_device_temp()
        self._zero_position: Final = self._position.value_as_double

        # Configure update frequencies
        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self._position,
            self._velocity,
            self._applied_volts,
            self._current,
            self._temperature
        )
        self._motor.optimize_bus_utilization()

        self._position_request = PositionVoltage(0)

    def update_inputs(self, inputs: ClimberIO.ClimberIOInputs) -> None:
        """Update inputs with current motor and servo state."""
        # Refresh all motor signals
        motor_status = BaseStatusSignal.refresh_all(
            self._position,
            self._velocity,
            self._applied_volts,
            self._current,
            self._temperature
        )

        # Update motor inputs
        inputs.climber_connected = motor_status.is_ok()
        inputs.climber_position = self._position.value_as_double
        inputs.climber_velocity = self._velocity.value_as_double
        inputs.climber_applied_volts = self._applied_volts.value_as_double
        inputs.climber_current = self._current.value_as_double
        inputs.climber_temperature = self._temperature.value_as_double
        inputs.climber_zero_position = self._zero_position

    def set_position(self, radians: float) -> None:
        """Set the motor position."""
        self._position_request.position = radiansToRotations(radians)
        self._position_request.position = self._position_request.position + self._zero_position
        self._motor.set_control(self._position_request)

class ClimberIOSim(ClimberIO):
    """
    Simulation implementation for testing without hardware.
    """

    def __init__(self) -> None:
        """Initialize the simulation IO."""
        self._motor_position: float = 0.0
        self._motor_velocity: float = 0.0
        self._motor_applied_volts: float = 0.0

        self._motor_type = DCMotor.krakenX60(1)
        self._climber_sim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self._motor_type,
                Constants.ClimberConstants.MOMENT_OF_INERTIA,
                Constants.ClimberConstants.GEAR_RATIO
            ),
            self._motor_type
        )

        self._closed_loop = False

        self._controller = PIDController(Constants.ClimberConstants.GAINS.k_p / (2*pi),
                                        Constants.ClimberConstants.GAINS.k_i / (2*pi),
                                        Constants.ClimberConstants.GAINS.k_d / (2*pi))

        self._zero_position = 0.0  # Sim starts at 0

    def update_inputs(self, inputs: ClimberIO.ClimberIOInputs) -> None:
        """Update inputs with simulated state."""
        # Simulate motor behavior (simple integration)
        # In a real simulation, you'd use a physics model here

        if self._closed_loop:
            self._motor_applied_volts = self._controller.calculate(self._climber_sim.getAngularPosition())
        else:
            self._controller.reset()

        self._climber_sim.setInputVoltage(max(-12.0, min(self._motor_applied_volts, 12.0)))
        self._climber_sim.update(0.02)  # 20ms periodic

        # Update inputs
        inputs.climber_connected = True
        inputs.climber_position = self._climber_sim.getAngularPosition()
        inputs.climber_velocity = self._climber_sim.getAngularAcceleration()
        inputs.climber_applied_volts = self._climber_sim.getInputVoltage()
        inputs.climber_current = abs(self._climber_sim.getCurrentDraw())  # Rough current estimate
        inputs.climber_temperature = 25.0  # Room temperature

    def set_open_loop(self, output):
        self._closed_loop = False
        self._motor_applied_volts = output

    def set_position(self, radians: float) -> None:
        self._closed_loop = True
        rotations = radiansToRotations(radians) + self._zero_position
        self._controller.setSetpoint(rotations)