from abc import ABC
from dataclasses import dataclass
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import PositionVoltage, VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, InvertedValue
from pykit.autolog import autolog
from wpimath.units import radians, radians_per_second, radiansToRotations, volts, amperes, celsius
from wpimath.system.plant import DCMotor, LinearSystemId
from wpilib.simulation import DCMotorSim
from wpimath.controller import PIDController
from math import pi

from constants import Constants
from util import tryUntilOk


class TurretIO(ABC):
    """
    Abstract base class for Turret IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass
    class TurretIOInputs:
        """Inputs from the Turret hardware/simulation."""
        # Motor status
        turret_connected: bool = False
        turret_position: radians = 0.0
        turret_velocity: radians_per_second = 0.0
        turret_applied_volts: volts = 0.0
        turret_current: amperes = 0.0
        turret_temperature: celsius = 0.0
        turret_setpoint: radians = 0.0
        turret_zero_position: float = 0.0
        turret_target_position: float = 0.0


    def update_inputs(self, inputs: TurretIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""
        pass

    def set_position(self, radians: float) -> None:
        """
        Set the turret position in radians.
        Args:
            radians: The position in radians to set the turret to.
        """
        pass
    
    def set_velocity(self, velocity: float) -> None:
        """
        Set the turret velocity in radians per second.
        Args:
            velocity: The velocity in radians per second to set the turret to.
        """
        pass


class TurretIOTalonFX(TurretIO):
    
    def __init__(self) -> None:
        self.turret_motor: Final[TalonFX] = TalonFX(Constants.CanIDs.TURRET_TALON, "rio")

        self.controller = PIDController(
            Constants.TurretConstants.GAINS.k_p,
            Constants.TurretConstants.GAINS.k_i,
            Constants.TurretConstants.GAINS.k_d,
            ) 

        motor_config = TalonFXConfiguration()
        motor_config.slot0 = Constants.TurretConstants.GAINS
        motor_config.feedback.sensor_to_mechanism_ratio = Constants.TurretConstants.GEAR_RATIO
        motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        motor_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        tryUntilOk(5, lambda: self.turret_motor.configurator.apply(motor_config, 0.25))

        self.position = self.turret_motor.get_position()
        self.velocity = self.turret_motor.get_velocity()
        self.applied_volts = self.turret_motor.get_motor_voltage()
        self.current = self.turret_motor.get_stator_current()
        self.temperature = self.turret_motor.get_device_temp()
        self.setpoint = self.turret_motor.get_closed_loop_reference()
        self._zero_position = self.position.value_as_double
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

        self.position_request = PositionVoltage(0)
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
        inputs.turret_zero_position = self._zero_position
        inputs.turret_target_position = self.target_position

    def set_position(self, radians: float) -> None:
        """
        Set the turret position in radians using closed loop control.
        Args:
            radians: The position in radians to set the turret to.
        """
        #commenting out for no jitter
        rotations = radiansToRotations(radians) + self._zero_position
        self.target_position = rotations
        if rotations > Constants.TurretConstants.MAX_ROTATIONS + self._zero_position:
            rotations = Constants.TurretConstants.MAX_ROTATIONS + self._zero_position
            print("Turret position is too high, setting to max")
        elif rotations < self._zero_position:
            rotations = self._zero_position
            print("Turret position is too low, setting to zero")
        self.position_request = PositionVoltage(rotations)
        self.turret_motor.set_control(self.position_request)

    def set_velocity(self, velocity: float) -> None:
        """
        Set the turret velocity in radians per second using closed loop control.
        Args:
            velocity: The velocity in radians per second to set the turret to.
        """
        if velocity > 0 and self.position.value_as_double >= Constants.TurretConstants.MAX_ROTATIONS + self._zero_position:
            velocity = 0
            print("Turret position is too high, setting to zero")
        elif velocity < 0 and self.position.value_as_double <= self._zero_position:
            velocity = 0
            print("Turret position is too low, setting to zero")
        self.velocity_request = VelocityVoltage(radiansToRotations(velocity))
        self.turret_motor.set_control(self.velocity_request)




class TurretIOSim(TurretIO):
    """
    Simulation implementation for testing without hardware.
    """

    def __init__(self) -> None:
        """Initialize the simulation IO."""
        self.motor = DCMotor.krakenX60(1)
        self.turretSim = DCMotorSim(LinearSystemId.DCMotorSystem(self.motor, Constants.TurretConstants.MOI, Constants.TurretConstants.GEAR_RATIO), self.motor)
        self.closed_loop = False

        self._motorPosition: float = 0.0
        self._motorVelocity: float = 0.0
        self.applied_volts: float = 0.0

        self.controller = PIDController(
            Constants.TurretConstants.GAINS.k_p / (2*pi),
            Constants.TurretConstants.GAINS.k_i / (2*pi),
            Constants.TurretConstants.GAINS.k_d / (2*pi),
            )

        self._zero_position = 0.0  # Sim starts at 0
        self.target_position = 0.0

    def update_inputs(self, inputs: TurretIO.TurretIOInputs) -> None:
        """Update inputs with simulated state."""

        if self.closed_loop:
            self.applied_volts = self.controller.calculate(self.turretSim.getAngularPosition())
        else:
            self.controller.reset()

        self.turretSim.setInputVoltage(max(-12.0, min(self.applied_volts, 12.0)))
        self.turretSim.update(.02)

        inputs.turret_connected = True
        inputs.turret_position = self.turretSim.getAngularPosition()
        inputs.turret_velocity = self.turretSim.getAngularAcceleration()
        inputs.turret_applied_volts = self.applied_volts
        inputs.turret_current = abs(self.turretSim.getCurrentDraw())
        inputs.turret_temperature = 25.0  # Room temperature


    def set_open_loop(self, output):
        self.closed_loop = False
        self.applied_volts = output

    def set_position(self, radians: float):
        """
        Set the turret position in radians.
        Args:
            radians: The position in radians to set the turret to.
        """
        self.closed_loop = True
        self.target_position = radiansToRotations(radians)
        self.controller.setSetpoint(radians)

    def set_velocity(self, velocity: float) -> None:
        """
        Set the turret velocity in radians per second.
        Args:
            velocity: The velocity in radians per second to set the turret to.
        """
        self.closed_loop = True
        if velocity > 0 and self._motorPosition*(2*pi) >= Constants.TurretConstants.MAX_ROTATIONS + self._zero_position:
            velocity = 0
        elif velocity < 0 and self._motorPosition*(2*pi) <= self._zero_position:
            velocity = 0
        self.controller.setSetpoint(velocity)   