"""IOs for the turret subsystem"""
from dataclasses import dataclass
from math import pi
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import MotionMagicVoltage, VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, InvertedValue
from pykit.autolog import autolog
from wpilib.simulation import DCMotorSim
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import (radians, radians_per_second, radiansToRotations,
                           volts, amperes, celsius)

from constants import Constants
from util import try_until_ok


class TurretIO:
    """
    Abstract base class for Turret IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    # pylint: disable=too-many-instance-attributes
    @autolog
    @dataclass(slots=True)
    class TurretIOInputs:
        """Inputs from the Turret hardware/simulation."""
        turret_connected: bool = False
        turret_position: radians = 0.0
        turret_velocity: radians_per_second = 0.0
        turret_applied_volts: volts = 0.0
        turret_current: amperes = 0.0
        turret_temperature: celsius = 0.0
        turret_setpoint: radians = 0.0
        turret_target_position: float = 0.0

    def update_inputs(self, inputs: TurretIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""

    def set_position(self, position: radians) -> None:
        """Set the turret position in position."""

    def set_velocity(self, velocity: radians_per_second) -> None:
        """Set the turret velocity in position per second."""

# pylint: disable=too-many-instance-attributes
class TurretIOTalonFX(TurretIO):
    """Phoenix6 TalonFX implementation."""

    def __init__(self) -> None:
        self.turret_motor: Final[TalonFX] = TalonFX(
            Constants.CanIDs.TURRET_TALON,
            "rio"
        )

        self.controller = PIDController(
            Constants.TurretConstants.GAINS.k_p,
            Constants.TurretConstants.GAINS.k_i,
            Constants.TurretConstants.GAINS.k_d,
        )

        motor_config = TalonFXConfiguration()
        motor_config.slot0 = Constants.TurretConstants.GAINS
        motor_config.motion_magic.motion_magic_cruise_velocity = (
            Constants.TurretConstants.MM_VELOCITY)
        motor_config.motion_magic.motion_magic_acceleration = (
            Constants.TurretConstants.MM_ACCELERATION)
        motor_config.feedback.sensor_to_mechanism_ratio = (
            Constants.TurretConstants.GEAR_RATIO)
        motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        motor_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        try_until_ok(
            5,
            lambda: self.turret_motor.configurator.apply(motor_config, 0.25)
        )
        try_until_ok(5,
            lambda: self.turret_motor.set_position(0)
        )

        self.position = self.turret_motor.get_position()
        self.velocity = self.turret_motor.get_velocity()
        self.applied_volts = self.turret_motor.get_motor_voltage()
        self.current = self.turret_motor.get_stator_current()
        self.temperature = self.turret_motor.get_device_temp()
        self.setpoint = self.turret_motor.get_closed_loop_reference()
        self.target_position = 0.0

        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self.position,
            self.velocity,
            self.applied_volts,
            self.current,
            self.temperature,
            self.setpoint
        )

        self.turret_motor.optimize_bus_utilization()

        self.position_request = MotionMagicVoltage(0)
        self.velocity_request = VelocityVoltage(0)

    def update_inputs(self, inputs: TurretIO.TurretIOInputs):
        motor_status = BaseStatusSignal.refresh_all(
            self.position,
            self.velocity,
            self.applied_volts,
            self.current,
            self.temperature,
            self.setpoint
        )

        inputs.turret_connected = motor_status.is_ok()
        inputs.turret_position = self.position.value_as_double
        inputs.turret_velocity = self.velocity.value_as_double
        inputs.turret_applied_volts = self.applied_volts.value_as_double
        inputs.turret_current = self.current.value_as_double
        inputs.turret_temperature = self.temperature.value_as_double
        inputs.turret_setpoint = self.setpoint.value_as_double
        inputs.turret_target_position = self.target_position

    def set_position(self, position: radians) -> None:
        """Set the turret position in position."""
        rotations = radiansToRotations(position)
        self.target_position = rotations
        self.turret_motor.set_control(
            self.position_request
            .with_position(rotations)
            .with_limit_forward_motion(rotations > Constants.TurretConstants.MAX_ROTATIONS)
            .with_limit_reverse_motion(rotations < 0)
        )

    def set_velocity(self, velocity: radians_per_second) -> None:
        """
        Set the turret velocity in position per second using closed loop
        control.
        """
        if (velocity > 0 and self.position.value_as_double >=
                Constants.TurretConstants.MAX_ROTATIONS):
            velocity = 0
        elif (velocity < 0 and self.position.value_as_double <=
              0):
            velocity = 0
        self.velocity_request = VelocityVoltage(radiansToRotations(velocity))
        self.turret_motor.set_control(self.velocity_request)

class TurretIOSim(TurretIO):
    """Simulation implementation for deterministic outputs."""

    def __init__(self) -> None:
        """Initialize the simulation IO."""
        self.motor = DCMotor.krakenX60(1)
        self.turret_sim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self.motor,
                Constants.TurretConstants.MOI,
                Constants.TurretConstants.GEAR_RATIO
            ),
            self.motor
        )
        self.closed_loop = False

        self._motor_position: float = 0.0
        self._motor_velocity: float = 0.0
        self.applied_volts: float = 0.0

        self.controller = PIDController(
            Constants.TurretConstants.GAINS.k_p / (2 * pi) * 5,
            Constants.TurretConstants.GAINS.k_i / (2 * pi),
            Constants.TurretConstants.GAINS.k_d / (2 * pi),
        )

        self.target_position = 0.0

    def update_inputs(self, inputs: TurretIO.TurretIOInputs) -> None:
        """Update inputs with simulated state."""

        if self.closed_loop:
            self.applied_volts = self.controller.calculate(
                self.turret_sim.getAngularPosition()
            )
        else:
            self.controller.reset()

        self.turret_sim.setInputVoltage(
            max(-12.0, min(self.applied_volts, 12.0))
        )
        self.turret_sim.update(0.02)

        inputs.turret_connected = True
        inputs.turret_position = self.turret_sim.getAngularPosition()
        inputs.turret_velocity = self.turret_sim.getAngularVelocity()
        inputs.turret_applied_volts = self.applied_volts
        inputs.turret_current = abs(self.turret_sim.getCurrentDraw())
        inputs.turret_temperature = 25.0  # Room temperature
        inputs.turret_setpoint = self.controller.getSetpoint()

    def set_open_loop(self, output: volts):
        """Set open loop voltage."""
        self.closed_loop = False
        self.applied_volts = output

    def set_position(self, position: radians):
        """Set the turret position in position."""
        self.closed_loop = True
        self.target_position = radiansToRotations(position)
        self.controller.setSetpoint(-position)

    def set_velocity(self, velocity: radians_per_second) -> None:
        """Set the turret velocity in position per second."""
        self.closed_loop = True
        if (velocity > 0 and self._motor_position * (
                2 * pi) >= Constants.TurretConstants.MAX_ROTATIONS):
            velocity = 0
        elif velocity < 0 and self._motor_position * (
                2 * pi) <= 0:
            velocity = 0
        self.controller.setSetpoint(-velocity)
