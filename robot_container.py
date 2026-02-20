import os
from typing import Optional

import commands2
import commands2.button
from commands2 import cmd, InstantCommand
from commands2.button import CommandXboxController, Trigger
from pathplannerlib.auto import NamedCommands, AutoBuilder, PathPlannerAuto
from pathplannerlib.util import FlippingUtil
from phoenix6 import swerve
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs, InvertedValue
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from wpilib import XboxController, getDeployDirectory
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from constants import Constants
from generated.larry.tuner_constants import TunerConstants as LarryTunerConstants
from generated.tuner_constants import TunerConstants
from robot_config import currentRobot, has_subsystem, Robot  # Robot detection (Larry vs Comp)
from subsystems.climber import ClimberSubsystem
from subsystems.climber.io import ClimberIOTalonFX, ClimberIOSim
from subsystems.intake import IntakeSubsystem
from subsystems.superstructure import Superstructure
from subsystems.swerve import SwerveSubsystem
from subsystems.vision import VisionSubsystem
from subsystems.hood import HoodSubsystem
from subsystems.hood.io import HoodIOSim, HoodIOTalonFX
from subsystems.turret import TurretSubsystem
from subsystems.turret.io import TurretIOTalonFX, TurretIOSim
import inspect


class RobotContainer:
    def __init__(self) -> None:
        # Log which robot we're running on (for debugging)
        print(f"Initializing RobotContainer for: {currentRobot.name}")
        self._max_speed = TunerConstants.speed_at_12_volts
        self._max_angular_rate = rotationsToRadians(1)
        if currentRobot == Robot.LARRY:
            self._max_speed = LarryTunerConstants.speed_at_12_volts

        self._driver_controller = commands2.button.CommandXboxController(0)
        self._function_controller = commands2.button.CommandXboxController(1)

        # Initialize subsystems as None - will be created conditionally
        self.climber: Optional[ClimberSubsystem] = None
        self.intake: Optional[IntakeSubsystem] = None
        self.drivetrain: Optional[SwerveSubsystem] = None
        self.vision: Optional[VisionSubsystem] = None
        self.turret: Optional[TurretSubsystem] = None
        match Constants.currentMode:
            case Constants.Mode.REAL:
                # Real robot, instantiate hardware IO implementations
                if has_subsystem("drivetrain"):
                    if currentRobot == Robot.LARRY:
                        self.drivetrain = LarryTunerConstants.create_drivetrain()
                    else:
                        self.drivetrain = TunerConstants.create_drivetrain()

                if has_subsystem("vision"):
                    self.vision = VisionSubsystem(
                        self.drivetrain,
                        Constants.VisionConstants.FRONT,
                    )

                if has_subsystem("turret"):
                    self.turret = TurretSubsystem(TurretIOTalonFX(), lambda: self.drivetrain.get_state().pose)

                # Create climber only if it exists on this robot
                if has_subsystem("climber"):
                    # Create climber motor config
                    # Note: Constants.ClimberConstants values are automatically selected based on detected robot
                    climber_motor_config = (TalonFXConfiguration()
                        .with_slot0(Constants.ClimberConstants.GAINS)
                        .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                        .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ClimberConstants.GEAR_RATIO))
                    )

                    # Create climber real hardware IO
                    # Note: Constants.CanIDs.CLIMB_TALON is automatically set based on detected robot (Larry vs Comp)
                    climber_io = ClimberIOTalonFX(
                        Constants.CanIDs.CLIMB_TALON,  # Different CAN ID for Larry vs Comp
                        Constants.ClimberConstants.SERVO_PORT,
                        climber_motor_config
                    )

                    # Create climber subsystem with real hardware IO
                    self.climber = ClimberSubsystem(climber_io)
                    print("Climber, Present")
                else:
                    print("Climber subsystem not available on this robot")

                    #create hood subsystem

                if has_subsystem("hood"):
                    hood_config = TalonFXConfiguration()
                    hood_config.slot0 = Constants.HoodConstants.GAINS
                    hood_config.feedback.sensor_to_mechanism_ratio = Constants.HoodConstants.GEAR_RATIO
                    hood_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
                    hood_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

                    hood_io = HoodIOTalonFX(
                        Constants.CanIDs.HOOD_TALON,
                    )

                    self.hood = HoodSubsystem(hood_io, lambda: self.drivetrain.get_state().pose)
                    print("we hood") # hood is present
                else:
                    print("straight out the suburbs") # hood is not present

            case Constants.Mode.SIM:
                # Sim robot, instantiate physics sim IO implementations (if available)
                self.drivetrain = TunerConstants.create_drivetrain()
                self.vision = VisionSubsystem(
                    self.drivetrain,
                    Constants.VisionConstants.FRONT,
                )
                #hood
                robot_pose_supplier = lambda: self.drivetrain.get_state().pose
                self.hood = HoodSubsystem(HoodIOSim(), robot_pose_supplier)

                self.turret = TurretSubsystem(TurretIOSim(), lambda: self.drivetrain.get_state().pose)

                # Create climber only if it exists on this robot
                if has_subsystem("climber"):
                    # Create climber subsystem with simulation IO
                    self.climber = ClimberSubsystem(ClimberIOSim())
                    print("Climber, Present")
                else:
                    print("Climber subsystem not available on this robot")

        self.superstructure = Superstructure(
            self.drivetrain, self.vision, self.climber, self.intake
        )

        self._setup_swerve_requests()
        self._pathplanner_setup()
        self._setup_controller_bindings()

    def set_robot_pose(self, command: commands2.Command) -> None:
        if isinstance(command, PathPlannerAuto):
            pose = command._startingPose
            if AutoBuilder.shouldFlip():
                pose = FlippingUtil.flipFieldPose(pose)
            self.drivetrain.reset_pose(pose)

    def _pathplanner_setup(self):
        # Register NamedCommands
        NamedCommands.registerCommand("Default", self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT))
        NamedCommands.registerCommand("Score in Hub", self.superstructure.set_goal_command(Superstructure.Goal.SCORE))
        NamedCommands.registerCommand("Climb Ready", self.superstructure.set_goal_command(Superstructure.Goal.CLIMBREADY))
        NamedCommands.registerCommand("Climb", self.superstructure.set_goal_command(Superstructure.Goal.CLIMB))

        # Build AutoChooser
        self._auto_chooser: LoggedDashboardChooser[commands2.Command] = LoggedDashboardChooser("Auto")

        for auto in os.listdir(os.path.join(getDeployDirectory(), 'pathplanner', 'autos')):
            auto = auto.removesuffix(".auto")
            if auto ==".DS_Store":
                continue
            self._auto_chooser.addOption(auto, PathPlannerAuto(auto, False))
            self._auto_chooser.addOption(auto + " (Mirrored)", PathPlannerAuto(auto, True))
        self._auto_chooser.setDefaultOption("None", cmd.none())
        self._auto_chooser.addOption("Basic Leave",
            self.drivetrain.apply_request(lambda: self._robot_centric.with_velocity_x(1)).withTimeout(1.0)
        )

        self._auto_chooser.onChange(self.set_robot_pose)

    def _setup_swerve_requests(self):
        self._field_centric = (
            swerve.requests.FieldCentric()
            .with_deadband(0)
            .with_rotational_deadband(0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
        )

        self._robot_centric: swerve.requests.RobotCentric = (
            swerve.requests.RobotCentric()
            .with_deadband(0)
            .with_rotational_deadband(0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
        )

        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

    @staticmethod
    def rumble_command(controller: CommandXboxController, duration: float, intensity: float):
        return cmd.sequence(
            InstantCommand(lambda: controller.setRumble(XboxController.RumbleType.kBothRumble, intensity)),
            cmd.waitSeconds(duration),
            InstantCommand(lambda: controller.setRumble(XboxController.RumbleType.kBothRumble, 0))
        )

    def _setup_controller_bindings(self) -> None:
        hid = self._driver_controller.getHID()

        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: self._field_centric
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
            )
        )

        self._driver_controller.leftBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._robot_centric
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
            )
        )

        if self.intake is not None:
            self._driver_controller.rightBumper().whileTrue(
                self.intake.set_desired_state_command(self.intake.SubsystemState.OUTPUT)
            ).onFalse(
                self.intake.set_desired_state_command(self.intake.SubsystemState.INTAKE)
            )
            self._driver_controller.b().whileTrue(
                self.intake.set_desired_state_command(self.intake.SubsystemState.STOP)
            ).onFalse(
                self.intake.set_desired_state_command(self.intake.SubsystemState.INTAKE)
            )
        else:
            print("Intake subsystem not available on this robot, unable to bind intake buttons")

        self._driver_controller.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._driver_controller.x().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(Rotation2d(-hid.getLeftY(), -hid.getLeftX()))
            )
        )

        self._driver_controller.start().onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.seed_field_centric()))

        goal_bindings = {
            self._function_controller.y(): self.superstructure.Goal.SCORE,
            self._function_controller.x(): self.superstructure.Goal.PASSDEPOT,
            self._function_controller.b(): self.superstructure.Goal.PASSOUTPOST,
            self._function_controller.a(): self.superstructure.Goal.DEFAULT,
            self._function_controller.povUp(): self.superstructure.Goal.CLIMBREADY,
            self._function_controller.povDown(): self.superstructure.Goal.CLIMB,
        }

        Trigger(lambda: self._function_controller.getLeftTriggerAxis() > 0.75).onTrue(
            self.vision.set_desired_state_command(self.vision.SubsystemState.NO_ESTIMATES)
        )

        Trigger(lambda: self._function_controller.getRightTriggerAxis() > 0.75).onTrue(
            self.vision.set_desired_state_command(self.vision.SubsystemState.ALL_ESTIMATES)
        )
        
        # Turret to hub
        self._function_controller.y().onTrue(self.turret.runOnce(lambda: self.turret.rotate_to_goal(self.turret.Goal.HUB)))

        # Turret to depot
        self._function_controller.x().onTrue(self.turret.runOnce(lambda: self.turret.rotate_to_goal(self.turret.Goal.DEPOT)))

        # Turret to outpost
        self._function_controller.b().onTrue(self.turret.runOnce(lambda: self.turret.rotate_to_goal(self.turret.Goal.OUTPOST)))

    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()

    def get_climber(self) -> Optional[ClimberSubsystem]:
        """Get the climber subsystem if it exists on this robot."""
        return self.climber

    def get_intake(self) -> Optional[IntakeSubsystem]:
        """Get the intake subsystem if it exists on this robot."""
        return self.intake

    def has_climber(self) -> bool:
        """Check if climber subsystem exists on this robot."""
        return self.climber is not None

    def has_intake(self) -> bool:
        """Check if intake subsystem exists on this robot."""
        return self.intake is not None
