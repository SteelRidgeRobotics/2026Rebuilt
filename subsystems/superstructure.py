"""Contains the superstructure, which handles subsystems states."""
import math
from enum import auto, IntEnum
from typing import Optional, Callable, TYPE_CHECKING

from commands2 import Command, Subsystem, cmd
from pathplannerlib.auto import AutoBuilder
from pykit.logger import Logger
from wpilib import DriverStation, Timer
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from constants import Constants
from subsystems.aiming import FiringTable
from subsystems.feeder import FeederSubsystem
from subsystems.hood import HoodSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.launcher import LauncherSubsystem
from subsystems.turret import TurretSubsystem

if TYPE_CHECKING:
    from subsystems.swerve import SwerveSubsystem


# pylint: disable=too-many-instance-attributes
class Superstructure(Subsystem):
    """
    The Superstructure is in charge of handling all subsystems to ensure no
    conflicts between them.
    """

    class Goal(IntEnum):
        """
        Superstructure goals.
        (Literally just SubsystemState but renamed)
        """
        DEFAULT = auto()  # Default goal
        INTAKE = auto()  # Intake fuel from the floor.
        LAUNCH = auto()  # Scoring fuel into the hub
        AIMHUB = auto()  # Point turret to hub
        AIMOUTPOST = auto()  # Point turret to the outpost side
        AIMDEPOT = auto()  # Point turret to the depot side
        # center

    # Map each goal to each subsystem state to reduce code complexity
    _goal_to_states: dict[Goal,
    tuple[
        Optional[IntakeSubsystem.SubsystemState],
        Optional[FeederSubsystem.SubsystemState],
        Optional[LauncherSubsystem.SubsystemState],
        Optional[HoodSubsystem.SubsystemState],
        Optional[TurretSubsystem.SubsystemState],
        bool,
        # Superstructure state? (Is it handled by periodic or just a single
        # action?)
    ]] = {

        Goal.DEFAULT: (
            IntakeSubsystem.SubsystemState.STOP,
            FeederSubsystem.SubsystemState.STOP,
            LauncherSubsystem.SubsystemState.IDLE,
            HoodSubsystem.SubsystemState.STOW,
            TurretSubsystem.SubsystemState.HUB,
            True
        ),

        Goal.INTAKE: (
            IntakeSubsystem.SubsystemState.INTAKE,
            FeederSubsystem.SubsystemState.STOP,
            LauncherSubsystem.SubsystemState.IDLE,
            HoodSubsystem.SubsystemState.STOW,
            None, True
        ),

        Goal.LAUNCH: (
            IntakeSubsystem.SubsystemState.INTAKE,
            FeederSubsystem.SubsystemState.INWARD,
            LauncherSubsystem.SubsystemState.SCORE,
            None, None, True
        ),

        Goal.AIMHUB: (
            None, None,
            LauncherSubsystem.SubsystemState.SCORE,
            HoodSubsystem.SubsystemState.AIMBOT,
            TurretSubsystem.SubsystemState.HUB,
            True  # track so aiming block runs and DistanceToHub is updated
        ),

        Goal.AIMOUTPOST: (
            None, None,
            LauncherSubsystem.SubsystemState.PASS,
            HoodSubsystem.SubsystemState.PASS,
            TurretSubsystem.SubsystemState.OUTPOST,
            True
        ),

        Goal.AIMDEPOT: (
            None, None,
            LauncherSubsystem.SubsystemState.PASS,
            HoodSubsystem.SubsystemState.PASS,
            TurretSubsystem.SubsystemState.DEPOT,
            True
        ),

    }

    # pylint: disable=too-many-arguments, too-many-positional-arguments
    def __init__(self,
                 intake: Optional[IntakeSubsystem] = None,
                 feeder: Optional[FeederSubsystem] = None,
                 launcher: Optional[LauncherSubsystem] = None,
                 hood: Optional[HoodSubsystem] = None,
                 turret: Optional[TurretSubsystem] = None,
                 drivetrain: Optional["SwerveSubsystem"] = None,
                 aim_pose_supplier: Optional[Callable[[], Pose2d]] = None,
                 firing_table: FiringTable = FiringTable()
                 ) -> None:
        """
        Constructs the superstructure using instance of each subsystem.
        Subsystems are optional to support robots that don't have all hardware.
        SOTM (shooting on the move): pass drivetrain, aim_pose_supplier,
        aiming_table to enable
        Virtual Goal aiming for LAUNCH and AIMHUB goals.
        """
        super().__init__()

        self.intake = intake
        self.feeder = feeder
        self.launcher = launcher
        self.hood = hood
        self.turret = turret
        self._drivetrain = drivetrain
        self._aim_pose_supplier = aim_pose_supplier
        self._firing_table = firing_table

        self._goal_state = self.Goal.DEFAULT
        self.set_goal_command(self._goal_state)

        self._turret_check = False
        self._hood_check = False
        self._flywheel_check = False
        self._checks_override = False

    # pylint: disable=too-many-branches
    def periodic(self):
        if DriverStation.isDisabled():
            return

        if (self._goal_state in (self.Goal.LAUNCH,
                                 self.Goal.AIMHUB) and
                self._aim_pose_supplier and self._firing_table):

            # Take the bloody shot
            sample = self._firing_table.get_moving_shot(
                self._aim_pose_supplier(),
                Constants.GoalLocations.RED_HUB
                if AutoBuilder.shouldFlip()
                else Constants.GoalLocations.BLUE_HUB,
                self._drivetrain.get_field_relative_speeds()
            )
            if self.turret:
                self.turret.set_target_field_angle(sample.turret_angle)
            sample = sample.sample # lovely line
            if self.hood:
                self.hood.set_aiming_setpoint(sample.hood_angle)
            if self.launcher:
                self.launcher.set_aiming_setpoint(sample.flywheel_speed)

        else:
            if self.turret:
                self.turret.set_target_field_angle(None)
            if self.hood:
                self.hood.set_aiming_setpoint(None)
            if self.launcher:
                self.launcher.set_aiming_setpoint(None)

        self._turret_check = (
            abs(
                self.turret.inputs.turret_setpoint -
                self.turret.inputs.turret_position
            ) < Constants.TurretConstants.SETPOINT_TOLERANCE
            if self.turret is not None else True
        )
        self._hood_check = (
            abs(
                self.hood.inputs.hood_setpoint - self.hood.inputs.hood_position
            ) < Constants.HoodConstants.SETPOINT_TOLERANCE
            if self.hood is not None else True
        )
        self._flywheel_check = (
            abs(
                self.launcher.desired_motor_rps -
                self.launcher.inputs.motor_velocity
            ) < Constants.LauncherConstants.SETPOINT_TOLERANCE
            if self.launcher is not None else True
        )

        match self._goal_state:
            case self.Goal.DEFAULT:
                if self.feeder.is_locked:
                    self.feeder.unlock()
                    self.feeder.set_desired_state(
                        FeederSubsystem.SubsystemState.STOP
                    )

            case self.Goal.INTAKE:
                if self.feeder.is_locked:
                    self.feeder.unlock()
                    self.feeder.set_desired_state(
                        FeederSubsystem.SubsystemState.INWARD
                    )

            case self.Goal.LAUNCH:
                if (
                        (
                                self._turret_check
                                and self._hood_check
                                and self._flywheel_check
                        ) or self._checks_override):
                    self.feeder.unlock()
                    self.feeder.set_desired_state(
                        FeederSubsystem.SubsystemState.INWARD
                    )
                else:
                    self.feeder.set_desired_state(
                        FeederSubsystem.SubsystemState.STOP
                    )
                    self.feeder.lock()

            case self.Goal.AIMHUB | self.Goal.AIMOUTPOST | self.Goal.AIMDEPOT:
                pass  # aiming block above handles setpoints; no feeder logic

        Logger.recordOutput("Superstructure/Goal State", self._goal_state.name)
        Logger.recordOutput("Superstructure/Turret Check", self._turret_check)
        Logger.recordOutput("Superstructure/Hood Check", self._hood_check)
        Logger.recordOutput(
            "Superstructure/Flywheel Check",
            self._flywheel_check
        )
        Logger.recordOutput("Superstructure/Overridden", self._checks_override)

    def _set_goal(self, goal: Goal) -> None:
        (intake_state, feeder_state, launcher_state, hood_state,
         turret_state, superstructure_state) = self._goal_to_states.get(
            goal,
            (None, None, None, None, None, False)
        )

        if not intake_state is None:
            self.intake.set_desired_state(intake_state)

        if not feeder_state is None:
            self.feeder.set_desired_state(feeder_state)

        if not launcher_state is None:
            self.launcher.set_desired_state(launcher_state)

        if not hood_state is None:
            self.hood.set_desired_state(hood_state)

        if not turret_state is None:
            self.turret.set_desired_state(turret_state)

        if superstructure_state:
            self._goal_state = goal

    def _toggle_override(self) -> None:
        self._checks_override = not self._checks_override

    def set_goal_command(self, goal: Goal) -> Command:
        """
        Return a command that sets the superstructure goal to whatever the
        desired goal is.

        :param goal: The desired goal
        :type goal:  Goal
        :return:     A command that will set the desired goal
        :rtype:      Command
        """
        return cmd.runOnce(lambda: self._set_goal(goal), self)

    def override_checks(self) -> Command:
        """Creates a command that toggles the check overrides."""
        return cmd.runOnce(self._toggle_override, self)
