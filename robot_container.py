import os
from typing import Callable, Optional

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
from wpilib import Field2d, SmartDashboard, XboxController, getDeployDirectory
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from constants import Constants
from generated.larry.tuner_constants import TunerConstants as LarryTunerConstants
from generated.tuner_constants import TunerConstants
from robot_config import currentRobot, has_subsystem, Robot  # Robot detection (Larry vs Comp)
from subsystems.climber import ClimberSubsystem
from subsystems.climber.io import ClimberIOTalonFX, ClimberIOSim
from subsystems.intake import IntakeSubsystem, IntakeIO, IntakeIOSim, IntakeIOTalonFX
from subsystems.superstructure import Superstructure
from subsystems.swerve import SwerveSubsystem
from subsystems.vision import VisionSubsystem
from subsystems.feeder import FeederIOSim, FeederIOTalonFX, FeederSubsystem
from subsystems.launcher import LauncherIOSim, LauncherIOTalonFX, LauncherSubsystem
from subsystems.hood import HoodSubsystem
from subsystems.hood.io import HoodIOSim, HoodIOTalonFX
from subsystems.turret import TurretSubsystem
from subsystems.turret.io import TurretIOTalonFX, TurretIOSim

from subsystems.vision.io import VisionIOLimelight


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

        # Field2d for Elastic dashboard (robot position on field image)
        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)

        # Initialize subsystems as None - will be created conditionally
        self.climber: Optional[ClimberSubsystem] = None
        self.intake: Optional[IntakeSubsystem] = None
        self.drivetrain: Optional[SwerveSubsystem] = None
        self.vision: Optional[VisionSubsystem] = None
        self.feeder: Optional[FeederSubsystem] = None
        self.launcher: Optional[LauncherSubsystem] = None
        self.turret: Optional[TurretSubsystem] = None
        self.hood: Optional[HoodSubsystem] = None

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
                        self.drivetrain.add_vision_measurement,
                        VisionIOLimelight(
                            Constants.VisionConstants.FRONT,
                            Constants.VisionConstants.robot_to_front,
                            lambda: self.drivetrain.get_state().pose.rotation(),
                        ),
                    )

                if has_subsystem("turret"):
                    turret_io = TurretIOTalonFX()
                    self.turret = TurretSubsystem(turret_io, lambda: self.drivetrain.get_state().pose)
                    print("Turret, present")
                else:
                    print("Turret subsystem not available on this robot")

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
                        climber_motor_config
                    )

                    # Create climber subsystem with real hardware IO
                    self.climber = ClimberSubsystem(climber_io)
                    print("Climber, Present")
                else:
                    print("Climber subsystem not available on this robot")

                if has_subsystem("feeder"):
                    feeder_io = FeederIOTalonFX()
                    self.feeder = FeederSubsystem(feeder_io)
                    print("Feeder, Present")
                else:
                    print("Feeder subsystem not available on this robot")

                if has_subsystem("launcher"):
                    launcher_io = LauncherIOTalonFX()
                    self.launcher = LauncherSubsystem(launcher_io, lambda: self.drivetrain.get_state().pose)
                    print("Launcher, Present")
                else:
                    print("Launcher subsystem not available on this robot")

                if has_subsystem("intake"):
                    intake_io = IntakeIOTalonFX()
                    self.intake = IntakeSubsystem(intake_io)
                    print("Intake, Present")
                else:
                    print("Intake subsystem not available on this robot")

                if has_subsystem("hood"):
                    hood_io = HoodIOTalonFX()

                    self.hood = HoodSubsystem(hood_io, lambda: self.drivetrain.get_state().pose)
                    print("we hood") # hood is present
                else:
                    print("straight out the suburbs") # hood is not present

            case Constants.Mode.SIM:
                # Sim robot, instantiate physics sim IO implementations (if available)
                self.drivetrain = TunerConstants.create_drivetrain()
                self.vision = VisionSubsystem(
                    self.drivetrain.add_vision_measurement,
                    VisionIOLimelight(
                        Constants.VisionConstants.FRONT,
                        Constants.VisionConstants.robot_to_front,
                        lambda: self.drivetrain.get_state().pose.rotation(),
                    ),
                )

                #hood
                robot_pose_supplier = lambda: self.drivetrain.get_state().pose
                self.hood = HoodSubsystem(HoodIOSim(), robot_pose_supplier)

                self.turret = TurretSubsystem(TurretIOSim(), lambda: self.drivetrain.get_state().pose)

                self.climber = ClimberSubsystem(ClimberIOSim())
                print("Climber, Present")

                if has_subsystem("feeder"):
                    self.feeder = FeederSubsystem(FeederIOSim())
                    print("Feeder, Present")
                else:
                    print("Feeder subsystem not available on this robot")

                if has_subsystem("launcher"):
                    self.launcher = LauncherSubsystem(LauncherIOSim(), lambda: self.drivetrain.get_state().pose)
                    print("Launcher, Present")
                else:
                    print("Launcher subsystem not available on this robot")

                if has_subsystem("intake"):
                    self.intake = IntakeSubsystem(IntakeIOSim())
                    print("Intake, Present")
                else:
                    print("Intake subsystem not available on this robot")

                if has_subsystem("turret"):
                    turret_io = TurretIOSim()
                    self.turret = TurretSubsystem(turret_io, lambda: self.drivetrain.get_state().pose)
                    print("Turret, present")
                else:
                    print("Turret subsystem not available on this robot")

                if has_subsystem("hood"):
                    hood_io = HoodIOSim()

                    self.hood = HoodSubsystem(hood_io, lambda: self.drivetrain.get_state().pose)
                    print("we hood") # hood is present
                else:
                    print("straight out the suburbs") # hood is not present

                

        self.superstructure = Superstructure(
            self.intake, self.feeder, self.launcher, self.hood
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
        NamedCommands.registerCommand("Launch", self.superstructure.set_goal_command(Superstructure.Goal.LAUNCH))
        NamedCommands.registerCommand("Aim to Depot", self.superstructure.set_goal_command(Superstructure.Goal.AIMDEPOT))
        NamedCommands.registerCommand("Aim to Outpost", self.superstructure.set_goal_command(Superstructure.Goal.AIMOUTPOST))
        NamedCommands.registerCommand("Aim to Hub", self.superstructure.set_goal_command(Superstructure.Goal.AIMHUB))

        #NamedCommands.registerCommand("Climber Extend", self.climber.set_desired_state(self.climber.SubsystemState.EXTEND))
        #NamedCommands.registerCommand("Climber Stow", self.climber.set_desired_state(self.climber.SubsystemState.STOW))

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
                InstantCommand(lambda: self.intake.set_desired_state(self.intake.SubsystemState.INTAKE))).onFalse(
                    InstantCommand(lambda: self.intake.set_desired_state(self.intake.SubsystemState.STOP)))

            Trigger(lambda: self._driver_controller.getRightTriggerAxis() > 0.75).whileTrue(
                InstantCommand(lambda: self.intake.set_desired_state(self.intake.SubsystemState.OUTPUT))
            ).onFalse(
                InstantCommand(lambda: self.intake.set_desired_state(self.intake.SubsystemState.STOP))
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

        if self.feeder is not None:
            self._function_controller.leftBumper().whileTrue(InstantCommand(lambda: self.feeder.set_desired_state(self.feeder.SubsystemState.INWARD))).onFalse(InstantCommand(lambda: self.feeder.set_desired_state(self.feeder.SubsystemState.STOP)))
        else:
            print("Feeder subsystem not available on this robot, unable to bind feeder buttons")
        if self.launcher is not None:
            self._function_controller.rightBumper().whileTrue(InstantCommand(lambda: self.launcher.set_desired_state(self.launcher.SubsystemState.SCORE))).onFalse(InstantCommand(lambda: self.launcher.set_desired_state(self.launcher.SubsystemState.IDLE)))
        else:
            print("Launcher subsystem not available on this robot, unable to bind launcher buttons")
        
        if self.turret is not None and self.hood is not None:
            self._function_controller.y().onTrue(
                InstantCommand(lambda: self.turret.set_desired_state(self.turret.SubsystemState.HUB)).alongWith(
                    InstantCommand(lambda: self.hood.set_desired_state(self.hood.SubsystemState.AIMBOT))
                )
            )

            self._function_controller.x().onTrue(
                InstantCommand(lambda: self.turret.set_desired_state(self.turret.SubsystemState.DEPOT)).alongWith(
                    InstantCommand(lambda: self.hood.set_desired_state(self.hood.SubsystemState.PASS))
                )
            )
            
            self._function_controller.b().onTrue(
                InstantCommand(lambda: self.turret.set_desired_state(self.turret.SubsystemState.OUTPOST)).alongWith(
                    InstantCommand(lambda: self.hood.set_desired_state(self.hood.SubsystemState.PASS))
                )
            )

            self._function_controller.a().onTrue(
                self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT)
            ).onFalse(self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT))

            Trigger(lambda: self._function_controller.getLeftTriggerAxis() > 0.75).onTrue(
                InstantCommand(lambda: self.turret.set_desired_state(self.turret.SubsystemState.MANUAL)).alongWith(
                    InstantCommand(lambda: self.hood.set_desired_state(self.hood.SubsystemState.MANUAL))
                )
            )
           

            Trigger(lambda: self._function_controller.getLeftTriggerAxis() > 0.75).whileTrue(
                InstantCommand(lambda: self.turret.rotate_manually(self._function_controller.getRightX()))
            )

            Trigger(lambda: self._function_controller.getLeftTriggerAxis() > 0.75).whileTrue(
                InstantCommand(lambda: self.hood.rotate_manually(self._function_controller.getRightY()))
            )
           
            #self._function_controller.getLeftTriggerAxis().onTrue(
        else:
            print("Turret or hood subsystem not available on this robot, unable to bind turret buttons")
        
        if self.hood is not None:
            self._function_controller.povDown().onTrue(
                InstantCommand(lambda: self.hood.set_desired_state(self.hood.SubsystemState.STOW))
            )
            self._function_controller.povUp().onTrue(
                InstantCommand(lambda: self.hood.set_desired_state(self.hood.SubsystemState.AIMBOT))
            )
        else:
            print("Hood subsystem not available on this robot, unable to bind hood buttons")

        if self.climber is not None:
            self._function_controller.povUp().onTrue(
                self.climber.set_desired_state_command(self.climber.SubsystemState.EXTEND)
            )
            self._function_controller.povDown().onTrue(
                self.climber.set_desired_state_command(self.climber.SubsystemState.STOW)
            )
        else:
            print("Climber subsystem not available on this robot, unable to bind climber buttons")


    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()

    def get_climber(self) -> Optional[ClimberSubsystem]:
        """Get the climber subsystem if it exists on this robot."""
        return self.climber

    def get_intake(self) -> Optional[IntakeSubsystem]:
        """Get the intake subsystem if it exists on this robot."""
        return self.intake

    def get_turret(self) -> Optional[TurretSubsystem]:
        """Get the turret subsystem if it exists on this robot."""
        return self.turret

    def get_hood(self) -> Optional[HoodSubsystem]:
        """Get the hood subsystem if it exists on this robot."""
        return self.hood

    def has_climber(self) -> bool:
        """Check if climber subsystem exists on this robot."""
        return self.climber is not None

    def has_intake(self) -> bool:
        """Check if intake subsystem exists on this robot."""
        return self.intake is not None

    def has_turret(self) -> bool:
        """Check if turret subsystem exists on this robot."""
        return self.turret is not None

    def has_hood(self) -> bool:
        """Check if hood subsystem exists on this robot."""
        return self.hood is not None
