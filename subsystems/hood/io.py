"""IO"""
from abc import ABC
from dataclasses import dataclass
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import PositionVoltage, VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue
from phoenix6.signals import NeutralModeValue
from phoenix6.units import celsius
from pykit.autolog import autolog
from wpilib.simulation import DCMotorSim
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor
from wpimath.system.plant import LinearSystemId
from wpimath.units import radians, radiansToRotations, volts, amperes, rotationsToRadians
from math import pi

from constants import Constants
from util import tryUntilOk


class HoodIO(ABC):
    """
    Abstract base class for hood IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass
    class HoodIOInputs:
        """Inputs from the hood hardware/simulation."""
        # Motor status
        hood_connected: bool = False
        hood_position: radians = 0.0
        hood_applied_volts: volts = 0.0
        hood_current: amperes = 0.0
        hood_temperature: celsius = 0.0
        hood_setpoint: float = 0.0
        hood_zero_position: float = 0.0


    def update_inputs(self, inputs: HoodIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""

    def set_position(self, rotation: float) -> None:
        """set rotation value (0-1) for the motor to go to."""

    def set_velocity(self, velocity: float) -> None:
        """
        Set the hood velocity in radians per second.
        Args:
            velocity: The velocity in radians per second to set the hood to.
        """

# pylint: disable=too-many-instance-attributes
class HoodIOTalonFX(HoodIO):
    """
    Real hardware implementation using TalonFX motor controller.
    """

    def __init__(self) -> None:
        """
        Initialize the real hardware IO.
        """

        self.hood_motor: Final[TalonFX] = TalonFX(Constants.CanIDs.HOOD_TALON, "rio")

        motor_config = TalonFXConfiguration()
        motor_config.slot0 = Constants.HoodConstants.GAINS
        motor_config.feedback.sensor_to_mechanism_ratio = Constants.HoodConstants.GEAR_RATIO
        motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        motor_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE


        # Apply motor configuration
        tryUntilOk(5, lambda: self.hood_motor.configurator.apply(motor_config, 0.25))


        # Create status signals for motor
        self.position = self.hood_motor.get_position()
        self.velocity = self.hood_motor.get_velocity()
        self.applied_volts = self.hood_motor.get_motor_voltage()
        self.current = self.hood_motor.get_stator_current()
        self.temperature = self.hood_motor.get_device_temp()
        self.setpoint = self.hood_motor.get_closed_loop_reference()
        self._zero_position = self.position.value_as_double
        self.target_position = self.position.value_as_double

        # Configure update frequencies
        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self.position,
            self.velocity,
            self.applied_volts,
            self.current,
            self.temperature,
            self.setpoint
        )
        self.hood_motor.optimize_bus_utilization()

        # Voltage control request
        self.position_request = PositionVoltage(0)
        self.velocity_request = VelocityVoltage(0)

    def update_inputs(self, inputs: HoodIO.HoodIOInputs) -> None:
        """Update inputs with current motor state."""
        # Refresh all motor signals
        motor_status = BaseStatusSignal.refresh_all(
            self.position,
            self.velocity,
            self.applied_volts,
            self.current,
            self.temperature,
            self.setpoint
        )

        # Update motor inputs
        inputs.hood_connected = motor_status.is_ok()
        inputs.hood_position = self.position.value_as_double
        inputs.hood_velocity = self.velocity.value_as_double
        inputs.hood_applied_volts = self.applied_volts.value_as_double
        inputs.hood_current = self.current.value_as_double
        inputs.hood_temperature = self.temperature.value_as_double
        inputs.hood_setpoint = self.setpoint.value_as_double
        inputs.hood_zero_position = self._zero_position

    def set_position(self, rotation: float) -> None:
        """Set the position."""
        if rotation > Constants.HoodConstants.MAX_ROTATIONS + self._zero_position:
            rotation = Constants.HoodConstants.MAX_ROTATIONS + self._zero_position
        elif rotation < self._zero_position:
            rotation = self._zero_position
        
        self.hood_motor.set_control(self.position_request.with_position(rotation))

    def set_velocity(self, velocity: float) -> None:
        """Set the velocity."""
        if velocity > 0 and self.position.value_as_double >= Constants.HoodConstants.MAX_ROTATIONS + self._zero_position:
            velocity = 0
        elif velocity < 0 and self.position.value_as_double <= self._zero_position:
            velocity = 0   
        self.velocity_request = VelocityVoltage(radiansToRotations(velocity))
        self.hood_motor.set_control(self.velocity_request)

class HoodIOSim(HoodIO):
    """Sim version of HoodIO."""

    def __init__(self) -> None:
        gearbox = DCMotor.krakenX44FOC(1)
        self.applied_volts = 0.0
        self.closed_loop = False

        self.hood_sim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                gearbox, 0.00783112228, Constants.HoodConstants.GEAR_RATIO),
            gearbox
        )

        self.controller = PIDController(
            Constants.HoodConstants.GAINS.k_p / (2*pi),
            Constants.HoodConstants.GAINS.k_i / (2*pi),
            Constants.HoodConstants.GAINS.k_d / (2*pi)
        )

        self._zero_position = 0.0  # Sim starts at 0

    def update_inputs(self, inputs: HoodIO.HoodIOInputs) -> None:
        """Update inputs with current motor Status Signals."""
        if self.closed_loop:
            self.applied_volts = (self.controller.calculate(self.hood_sim.getAngularPosition()))
        else:
            self.controller.reset()

        self.hood_sim.setInputVoltage(max(-12.0, min(self.applied_volts, 12.0)))

        self.hood_sim.update(0.02)
        inputs.hood_connected = True
        inputs.hood_applied_volts = self.applied_volts
        inputs.hood_position = self.hood_sim.getAngularPosition()
        inputs.hood_velocity = self.hood_sim.getAngularVelocity()
        inputs.hood_current = abs(self.hood_sim.getCurrentDraw())

    def set_open_loop(self, output: volts) -> None:
        """Set the open loop output."""
        self.closed_loop = False
        self.applied_volts = output

    def set_position(self, rotation: float) -> None:
        """Set the position."""

        if rotation > Constants.HoodConstants.MAX_ROTATIONS + self._zero_position:
            rotation = Constants.HoodConstants.MAX_ROTATIONS + self._zero_position
        elif rotation < self._zero_position:
            rotation = self._zero_position

        self.closed_loop = True
        self.controller.setSetpoint(rotationsToRadians(rotation))

    def set_velocity(self, velocity: float) -> None:
        self.closed_loop = True
        self.controller.setSetpoint(velocity)
