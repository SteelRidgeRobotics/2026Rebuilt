"""Big ole container"""
import os
from typing import Optional

import commands2
import commands2.button
from commands2 import cmd, InstantCommand
from commands2.button import Trigger
from pathplannerlib.auto import NamedCommands, AutoBuilder, PathPlannerAuto
from pathplannerlib.util import FlippingUtil
from phoenix6 import swerve
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import (NeutralModeValue,
                                            MotorOutputConfigs,
                                            FeedbackConfigs)
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from wpilib import (Field2d, SmartDashboard, getDeployDirectory, RobotBase)
from wpimath.geometry import Rotation2d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import rotationsToRadians, inchesToMeters

from constants import Constants
from generated.larry.tuner_constants import (TunerConstants as
                                             LarryTunerConstants)
from generated.tuner_constants import TunerConstants
from lib.fuel_sim import FuelSim
from robot_config import currentRobot, has_subsystem, Robot
from subsystems import superstructure
from subsystems.aiming import ShooterAimingTable
from subsystems.climber import ClimberSubsystem
from subsystems.climber.io import ClimberIOTalonFX, ClimberIOSim, ClimberIO
from subsystems.feeder import (FeederIOSim, FeederIOTalonFX, FeederSubsystem,
                               FeederIO)
from subsystems.hood import HoodSubsystem
from subsystems.hood.io import HoodIOSim, HoodIOTalonFX, HoodIO
from subsystems.intake import (IntakeSubsystem, IntakeIOSim, IntakeIOTalonFX,
                               IntakeIO)
from subsystems.launcher import (LauncherIOSim, LauncherIOTalonFX,
                                 LauncherSubsystem, LauncherIO)
from subsystems.superstructure import Superstructure
from subsystems.swerve import SwerveSubsystem
from subsystems.vision import VisionSubsystem
from subsystems.vision.io import VisionIOLimelight, VisionIO


# pylint: disable=too-many-instance-attributes
class RobotContainer:
    """Handles all subsystems and controller bindings."""

    def __init__(self) -> None:
        # Log which robot we're running on (for debugging)
        print(f"Initializing RobotContainer for: {currentRobot.name}")
        self._max_speed = TunerConstants.speed_at_12_volts
        self._max_angular_rate = rotationsToRadians(1)

        if currentRobot == Robot.LARRY:
            self._max_speed = LarryTunerConstants.speed_at_12_volts

        self._driver_controller = commands2.button.CommandXboxController(0)

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
        self.hood: Optional[HoodSubsystem] = None

        match Constants.CURRENT_MODE:
            case Constants.Mode.REAL:
                # Real robot, instantiate hardware IO implementations
                if currentRobot == Robot.LARRY:
                    self.drivetrain = (
                        LarryTunerConstants.create_drivetrain())
                else:
                    self.drivetrain = TunerConstants.create_drivetrain()

                self.vision = VisionSubsystem(
                    self.drivetrain.add_vision_measurement,
                    VisionIOLimelight(
                        Constants.VisionConstants.FRONT,
                        Constants.VisionConstants.robot_to_front,
                        lambda: self.drivetrain.get_cached_state(
                        ).pose.rotation(),
                    ),
                    VisionIOLimelight(
                        Constants.VisionConstants.BACK,
                        Constants.VisionConstants.robot_to_back,
                        lambda: self.drivetrain.get_cached_state(
                        ).pose.rotation(),
                    ),
                )

                # Hood, launcher, turret use turret position (robot center +
                # offset) for distance/aim
                # Create climber only if it exists on this robot
                if has_subsystem("climber"):
                    # Create climber motor config
                    # Note: Constants.ClimberConstants values are
                    # automatically selected based on detected robot
                    climber_motor_config = (TalonFXConfiguration()
                    .with_slot0(
                        Constants.ClimberConstants.GAINS
                    )
                    .with_motor_output(
                        MotorOutputConfigs().with_neutral_mode(
                            NeutralModeValue.BRAKE
                        )
                    )
                    .with_feedback(
                        FeedbackConfigs().with_sensor_to_mechanism_ratio(
                            Constants.ClimberConstants.GEAR_RATIO
                        )
                    )
                    )

                    # Create climber real hardware IO
                    # Note: Constants.CanIDs.CLIMB_TALON is automatically
                    # set based on detected robot (Larry vs Comp)
                    climber_io = ClimberIOTalonFX(
                        Constants.CanIDs.CLIMB_TALON,
                        # Different CAN ID for Larry vs Comp
                        climber_motor_config
                    )

                    # Create climber subsystem with real hardware IO
                    self.climber = ClimberSubsystem(climber_io)

                if has_subsystem("feeder"):
                    feeder_io = FeederIOTalonFX()
                    self.feeder = FeederSubsystem(feeder_io)

                if has_subsystem("launcher"):
                    launcher_io = LauncherIOTalonFX()
                    self.launcher = LauncherSubsystem(
                        launcher_io,
                        lambda: self.drivetrain.get_cached_state().pose
                    )

                if has_subsystem("intake"):
                    intake_io = IntakeIOTalonFX()
                    self.intake = IntakeSubsystem(intake_io)

                if has_subsystem("hood"):
                    hood_io = HoodIOTalonFX()
                    self.hood = HoodSubsystem(
                        hood_io,
                        lambda: self.drivetrain.get_cached_state().pose
                    )

            case Constants.Mode.SIM:
                # Drivetrain has no IOs, initialize like normal
                self.drivetrain = TunerConstants.create_drivetrain()
                self.vision = VisionSubsystem(
                    self.drivetrain.add_vision_measurement,
                    VisionIOLimelight(
                        Constants.VisionConstants.FRONT,
                        Constants.VisionConstants.robot_to_front,
                        lambda: self.drivetrain.get_cached_state(

                        ).pose.rotation(),
                    ),
                )

                # hood, turret, launcher use turret position (robot center +
                # offset) for distance/aim
                self.hood = HoodSubsystem(HoodIOSim(), lambda: self.drivetrain.get_cached_state().pose) # TODO Run in sim to make sure it works

                self.climber = ClimberSubsystem(ClimberIOSim())

                if has_subsystem("feeder"):
                    self.feeder = FeederSubsystem(FeederIOSim())

                if has_subsystem("launcher"):
                    self.launcher = LauncherSubsystem(
                        LauncherIOSim(),
                        lambda: self.drivetrain.get_cached_state().pose
                    )

                if has_subsystem("intake"):
                    self.intake = IntakeSubsystem(IntakeIOSim())

                if has_subsystem("hood"):
                    hood_io = HoodIOSim()
                    self.hood = HoodSubsystem(hood_io, lambda: self.drivetrain.get_cached_state().pose)

            case Constants.Mode.REPLAY:
                # Initialize all subsystems
                self.drivetrain = TunerConstants.create_drivetrain()
                self.vision = VisionSubsystem(
                    self.drivetrain.add_vision_measurement,
                    VisionIO()
                )
                self.hood = HoodSubsystem(HoodIO(), lambda: self.drivetrain.get_cached_state().pose)
                self.climber = ClimberSubsystem(ClimberIO())
                self.intake = IntakeSubsystem(IntakeIO())
                self.feeder = FeederSubsystem(FeederIO())
                self.launcher = LauncherSubsystem(LauncherIO(), lambda: self.drivetrain.get_cached_state().pose)

        # Fuel simulation (for simulation testing)
        def get_field_speeds():
            state = self.drivetrain.get_cached_state()
            speeds = state.speeds
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                speeds.vx,
                speeds.vy,
                speeds.omega,
                state.pose.rotation()
            )

        self.fuel_sim = FuelSim()
        if RobotBase.isSimulation():
            self.fuel_sim.spawn_starting_fuel()
            self.fuel_sim.register_robot(
                inchesToMeters(27),
                inchesToMeters(27),
                inchesToMeters(5),
                lambda: self.drivetrain.get_cached_state().pose,
                get_field_speeds
            )
            self.fuel_sim.enable_air_resistance()
            self.fuel_sim.start()

        self.superstructure = Superstructure(
            self.intake,
            self.feeder,
            self.launcher,
            self.hood,
            drivetrain=self.drivetrain,
            aim_pose_supplier=(
                lambda: self.drivetrain.get_cached_state().pose if self.drivetrain is not None else None
            ),
            aiming_table=ShooterAimingTable(),
        )

        self._setup_swerve_requests()
        self._pathplanner_setup()
        self._setup_controller_bindings()

    def set_robot_pose(self, command: commands2.Command) -> None:
        """Set the robot pose to the start of the auto"""
        if isinstance(command, PathPlannerAuto):
            # pylint: disable=protected-access
            pose = command._startingPose
            if AutoBuilder.shouldFlip():
                pose = FlippingUtil.flipFieldPose(pose)
            self.drivetrain.reset_pose(pose)

    def _pathplanner_setup(self):
        # Register NamedCommands
        NamedCommands.registerCommand(
            "Default",
            self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT)
        )
        NamedCommands.registerCommand(
            "Launch",
            self.superstructure.set_goal_command(Superstructure.Goal.LAUNCH).alongWith(self.intake.set_desired_state_command(self.intake.SubsystemState.INTAKE))
        )
        NamedCommands.registerCommand(
            "Aim to Depot",
            self.superstructure.set_goal_command(Superstructure.Goal.AIMDEPOT)
        )
        NamedCommands.registerCommand(
            "Aim to Outpost",
            self.superstructure.set_goal_command(
              Superstructure.Goal.AIMOUTPOST
            )
        )
        NamedCommands.registerCommand(
            "Aim to Hub",
            self.superstructure.set_goal_command(Superstructure.Goal.AIMHUB)
        )
        NamedCommands.registerCommand(
            "Intake",
            self.superstructure.set_goal_command(Superstructure.Goal.INTAKE)
        )
        NamedCommands.registerCommand(
            "Climber Extend",
            cmd.none()
        )
        NamedCommands.registerCommand(
            "Climber Stow",
            cmd.none()
        )

        # Build AutoChooser
        self._auto_chooser: LoggedDashboardChooser[
            commands2.Command] = LoggedDashboardChooser("Auto")

        for auto in os.listdir(
                os.path.join(getDeployDirectory(), 'pathplanner', 'autos')
        ):
            auto = auto.removesuffix(".auto")
            if auto == ".DS_Store":
                continue
            print(f'Adding "{auto}"...')
            self._auto_chooser.addOption(auto, PathPlannerAuto(auto, False))
            self._auto_chooser.addOption(
                auto + " (Mirrored)",
                PathPlannerAuto(auto, True)
            )
        self._auto_chooser.setDefaultOption("None", cmd.none())
        self._auto_chooser.addOption(
            "Basic Leave",
            self.drivetrain.apply_request(
                lambda: self._robot_centric.with_velocity_x(1)
            ).withTimeout(1.0)
        )

        self._auto_chooser.onChange(self.set_robot_pose)


        PPHolonomicDriveController.setRotationTargetOverride(
            self._auto_aim_heading_override
        )

    def _setup_swerve_requests(self):
        self._field_centric = (
            swerve.requests.FieldCentric()
            .with_deadband(0)
            .with_rotational_deadband(0)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
            .with_steer_request_type(
                swerve.SwerveModule.SteerRequestType.POSITION
            )
        )

        self._robot_centric: swerve.requests.RobotCentric = (
            swerve.requests.RobotCentric()
            .with_deadband(0)
            .with_rotational_deadband(0)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
            .with_steer_request_type(
                swerve.SwerveModule.SteerRequestType.POSITION
            )
        )
        self._field_centric_facing_angle = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.01)
            .with_rotational_deadband(self._max_angular_rate * 0.02)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
            .with_steer_request_type(
                swerve.SwerveModule.SteerRequestType.POSITION
            )
            .with_heading_pid(
                Constants.AutoAlignConstants.HEADING_KP,
                Constants.AutoAlignConstants.HEADING_KI,
                Constants.AutoAlignConstants.HEADING_KD,
            )
            .with_max_abs_rotational_rate(self._max_angular_rate)
        )
        

        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()


    def _aim_field_heading(self) -> Optional[Rotation2d]:
        """Field heading to face while aiming, or None when driver controls yaw."""
        if not self.superstructure.is_chassis_aiming():
            return None
        if not self.superstructure._heading_on_target():
            return self.superstructure.get_target_pose(self.drivetrain.get_cached_state().pose)
        return None

    def _auto_aim_heading_override(self) -> Optional[Rotation2d]:
        """PathPlanner rotation override so autos face the goal while aiming."""
        return self._aim_field_heading()

    def _teleop_drive_request(self, hid) -> swerve.requests.SwerveRequest:
        """Field-centric drive; lock heading to the goal while aiming."""
        velocity_x = -hid.getLeftY() * self._max_speed
        velocity_y = -hid.getLeftX() * self._max_speed
        heading = self._aim_field_heading()
        if heading is not None:
            # FieldCentricFacingAngle with operator perspective rotates the
            # target by alliance forward, so convert field heading first.
            operator_forward =  self.drivetrain.get_operator_forward_direction()
            return (
                self._field_centric_facing_angle
                .with_velocity_x(velocity_x)
                .with_velocity_y(velocity_y)
                .with_target_direction(heading)

                )
            
        return (
            self._field_centric
            .with_velocity_x(velocity_x)
            .with_velocity_y(velocity_y)
            .with_rotational_rate(
                -self._driver_controller.getRightX() * self._max_angular_rate
            )
        )

    def _setup_controller_bindings(self) -> None:
        hid = self._driver_controller.getHID()

        self.drivetrain.setDefaultCommand(self.drivetrain.apply_request(lambda: self._teleop_drive_request(hid)))
        
        

        self._driver_controller.leftBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._robot_centric
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(
                    -self._driver_controller.getRightX() *
                    self._max_angular_rate
                )
            )
        )

        if self.intake is not None:
            self._driver_controller.rightBumper().whileTrue(
                InstantCommand(
                    lambda: self.intake.set_desired_state(
                        self.intake.SubsystemState.INTAKE
                    )
                )
            ).onFalse(
                InstantCommand(
                    lambda: self.intake.set_desired_state(
                        self.intake.SubsystemState.STOP
                    )
                )
            )

            self._driver_controller.rightBumper().whileTrue(
                InstantCommand(
                    lambda: self.intake.set_desired_state(
                        self.intake.SubsystemState.OUTPUT
                    )
                )
            ).onFalse(
                InstantCommand(
                    lambda: self.intake.set_desired_state(
                        self.intake.SubsystemState.STOP
                    )
                )
            )
        else:
            print(
                "Intake subsystem not available on this robot, unable to "
                "bind intake buttons"
            )

        self._driver_controller.povRight().whileTrue(
            self.drivetrain.apply_request(lambda: self._brake)
        )
        self._driver_controller.povLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-hid.getLeftY(), -hid.getLeftX())
                )
            )
        )

        self._driver_controller.start().onTrue(
            self.drivetrain.runOnce(
                self.drivetrain.seed_field_centric
            )
        )

        if self.launcher is not None:
            Trigger(
                lambda: self._driver_controller.getRightTriggerAxis() > 0.75
            ).whileTrue(
                self.superstructure.set_goal_command(
                    Superstructure.Goal.LAUNCH
                )
            ).onFalse(
                self.superstructure.set_goal_command(
                    Superstructure.Goal.STOPLAUNCH
                )
            )
            Trigger(
                lambda: self._driver_controller.getLeftTriggerAxis() > 0.75
            ).whileTrue(
                InstantCommand(
                    lambda: self.intake.set_desired_state(
                        self.intake.SubsystemState.INTAKE
                    )
                )
            ).onFalse(
                InstantCommand(
                    lambda: self.intake.set_desired_state(
                        self.intake.SubsystemState.STOP
                    )
                )
            )

        else:
            print(
                "Launcher subsystem not available on this robot, unable to "
                "bind launcher buttons"
            )

        if self.hood is not None:

            self._driver_controller.y().onTrue(
                self.superstructure.set_goal_command(
                    Superstructure.Goal.AIMHUB
                )
            )

            self._driver_controller.x().onTrue(
                self.superstructure.set_goal_command(
                    Superstructure.Goal.AIMDEPOT
                )
            )

            self._driver_controller.b().onTrue(
                self.superstructure.set_goal_command(
                    Superstructure.Goal.AIMOUTPOST
                )
            )

            self._driver_controller.a().onTrue(
                self.superstructure.set_goal_command(
                    Superstructure.Goal.DEFAULT
                )
            )


        else:
            print(
                "Turret or hood subsystem not available on this robot, "
                "unable to bind turret buttons"
            )

            
            
            

        if self.climber is not None:
            self._driver_controller.povUp().onTrue(
                self.climber.set_desired_state_command(
                    self.climber.SubsystemState.EXTEND
                )
            )
            self._driver_controller.povDown().onTrue(
                self.climber.set_desired_state_command(
                    self.climber.SubsystemState.STOW
                )
            )
        else:
            print(
                "Climber subsystem not available on this robot, unable to "
                "bind climber buttons"
            )

    def get_autonomous_command(self) -> commands2.Command:
        """Returns the selected auto command."""
        return self._auto_chooser.getSelected()

    def get_climber(self) -> Optional[ClimberSubsystem]:
        """Get the climber subsystem if it exists on this robot."""
        return self.climber

    def get_intake(self) -> Optional[IntakeSubsystem]:
        """Get the intake subsystem if it exists on this robot."""
        return self.intake

    def get_hood(self) -> Optional[HoodSubsystem]:
        """Get the hood subsystem if it exists on this robot."""
        return self.hood

    def has_climber(self) -> bool:
        """Check if climber subsystem exists on this robot."""
        return self.climber is not None

    def has_intake(self) -> bool:
        """Check if intake subsystem exists on this robot."""
        return self.intake is not None

    def has_hood(self) -> bool:
        """Check if hood subsystem exists on this robot."""
        return self.hood is not None

    def get_component_poses(self) -> list[Pose3d]:
        """Gets component poses for AdvantageScope logging."""
        hood_pose = Pose3d() # TODO this position might be off due to turret being removed

        if self.climber is not None:
            climber_pose = self.climber.get_component_pose()
        else:
            climber_pose = Pose3d()

        return [hood_pose, climber_pose]