from enum import Enum, auto
from typing import Final
from math import *

from phoenix6.configs.config_groups import Slot0Configs
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import RobotBase
from wpimath.geometry import Pose2d, Transform3d, Rotation2d, Rotation3d

from robot_config import currentRobot, Robot


class Constants:

    class Mode(Enum):
        # Running on a real robot.
        REAL = auto(),

        # Running a physics simulator.
        SIM = auto(),

        # Replaying from a log file.
        REPLAY = auto()

    simMode: Final[Mode] = Mode.SIM
    currentMode: Final[Mode] = Mode.REAL if RobotBase.isReal() else simMode

    FIELD_LAYOUT: Final[AprilTagFieldLayout] = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    # Hardware configurations
    # Can ids are to be set in the same order as they are wired in the CAN bus
    class CanIDs:
        #All motors are Kraken X60 unless otherwise specified
        INTAKE_TALON = 10
        CLIMB_TALON = 11
        TURRET_TALON = 12
        FEEDER_TALON = 13
        HOOD_TALON = 14 # Kraken X44
        LAUNCHER_TOP_TALON = 15 # Kraken X44
        LAUNCHER_LOW_TALON = 16 # Kraken X44

        # Power Distribution (REV PDH or CTRE PDP). Set to None if no PDH/PDP on CAN bus
        # to avoid "CAN: Message not found: Module N" errors from pykit logging.
        POWER_DISTRIBUTION_MODULE_ID: Final[int | None] = None

    class GeneralConstants:
        GAME_PIECE_WEIGHT = 0.215

    class ClimberConstants:
        GEAR_RATIO = 61504.0 / 189
        GAINS = (Slot0Configs()
                .with_k_p(40.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
        VOLTAGE_INWARDS = None
        VOLTAGE_OUTWARDS = None
        CLIMB_FULL_THRESHOLD = 5.0 # Adjust as needed
        SUPPLY_CURRENT = 30.0
        MOMENT_OF_INERTIA = 0.3 # Placeholder until climber is finished

    class IntakeConstants:
        GEAR_RATIO = None
        GAINS: Slot0Configs = None
        GAINS = None
        SUPPLY_CURRENT = None
        MOMENT_OF_INERTIA = None
        FEED_FORWARD = None

    class LauncherConstants:
        GEAR_RATIO = None
        GAINS: Slot0Configs = None
        SUPPLY_CURRENT = None
        MOMENT_OF_INERTIA = None
        FLYWHEEL_RADIUS = None
        FEED_FORWARD = None
        MAX_RPS = None


    class FeederConstants:
        GEAR_RATIO = None
        GAINS: Slot0Configs = None
        SUPPLY_CURRENT = None
        MOMENT_OF_INERTIA = None
        FEED_FORWARD = None

    class VisionConstants:
        FRONT = "limelight-front"
        # LAUNCHER = "limelight-al"

        robot_to_front = Transform3d(
            0.2,
            0.0,
            0.0,
            Rotation3d(
                0.0,
                0.0,
                0.0
            )
        )

        max_ambiguity = 0.3
        max_z_error = 0.75

        # StdDev baselines, for 1-meter distance and 1 tag
        # (Adjusted automatically based on distance and # of tags)
        linear_std_dev_baseline = 0.02 # Meters
        angular_std_dev_baseline = 0.06 # Radians

    class TurretConstants:
        GAINS = (Slot0Configs()
                .with_k_p(14.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.5)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
        GEAR_RATIO = 170/36
        SUPPLY_CURRENT = 40
        MOI = .455
        MAX_ROTATIONS = 0.865967
        MAX_MANUAL_VELOCITY = 20  # rad/sec
        # Only switch to other side of center when goal is at least this many degrees past middle
        CROSS_MIDDLE_HYSTERESIS_DEGREES = 5.0

    class HoodConstants:
        GEAR_RATIO = 68/3
        GAINS = (Slot0Configs()
                .with_k_p(65.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.38)
                .with_k_v(0.0)
                .with_k_a(0.0)
        )
        SUPPLY_CURRENT = 30
        # positions
        STOW = -0.0263671875
        PASSING = 0.09
        MAX_MANUAL_VELOCITY = 20
        HARDCODED_POSITION = 0.05
        MAX_ROTATIONS = 0.054687567
        MAX_DISTANCE_FOR_SLOW_LAUNCH = 2.55
        MAX_DISTANCE_FOR_MEDIUM_LAUNCH = 2.90

    class FieldConstants:
        HUB_POSE = Pose2d(4.625594, 4.034536, 0.0)  # blue hub, flip when needed
        HUB_HEIGHT = 1.3860018  # hub height - initial height of shooter (17.433 inches) (in meters)    class TurretConstants:

    class GoalLocations:
        BLUE_HUB = Pose2d(4.625594, 4.034536, 0)
        BLUE_DEPOT_PASS = Pose2d(4.020286, 0, 0)
        BLUE_OUTPOST_PASS = Pose2d(4.020286, 8.069072, 0)

        RED_HUB = Pose2d(11.915394, 4.034536, 180)
        RED_DEPOT_PASS = Pose2d(12.517591, 8.069072, 180)
        RED_OUTPOST_PASS = Pose2d(12.517591, 0, 180)




# Initialize robot-specific hardware configurations
def _init_hardware_configs():
    """Initialize hardware configurations based on detected robot."""

    match currentRobot:
        case Robot.LARRY:
            # Climber
            Constants.ClimberConstants.GEAR_RATIO = 61504.0 / 189
            Constants.ClimberConstants.GAINS = (Slot0Configs()
                .with_k_p(40.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
            Constants.ClimberConstants.VOLTAGE_INWARDS = 16.0
            Constants.ClimberConstants.VOLTAGE_OUTWARDS = -4.0
            Constants.ClimberConstants.CLIMB_FULL_THRESHOLD = 5.0  # Adjust as needed
            Constants.ClimberConstants.SUPPLY_CURRENT = 30.0
            Constants.ClimberConstants.MOMENT_OF_INERTIA = 0.3

        case _:  # COMP or UNKNOWN defaults to COMP
            # Climber
            Constants.ClimberConstants.GEAR_RATIO = 61504.0 / 189  # Same or different?
            Constants.ClimberConstants.GAINS = (Slot0Configs()
                .with_k_p(40.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
            Constants.ClimberConstants.VOLTAGE_INWARDS = 16.0
            Constants.ClimberConstants.VOLTAGE_OUTWARDS = -4.0
            Constants.ClimberConstants.CLIMB_FULL_THRESHOLD = 5.0  # Adjust as needed
            Constants.ClimberConstants.SUPPLY_CURRENT = 30.0
            Constants.ClimberConstants.MOMENT_OF_INERTIA = 0.3

            # Intake
            Constants.IntakeConstants.GEAR_RATIO = 3.8333  # Adjust based on actual gear ratio
            Constants.IntakeConstants.GAINS = (Slot0Configs()
                .with_k_p(0.5)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.17)
                .with_k_v(0.14)
                .with_k_a(0.0)
            )
            Constants.IntakeConstants.SUPPLY_CURRENT = 30.0  # Amperes
            Constants.IntakeConstants.MOMENT_OF_INERTIA = 0.0067

            # Launcher
            Constants.LauncherConstants.GEAR_RATIO = 1.25  # Adjust based on actual gear ratio
            Constants.LauncherConstants.GAINS = (Slot0Configs()
                .with_k_p(0.2)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.2750000067)
                .with_k_v(0.12787)
                .with_k_a(0.0)
            )
            Constants.LauncherConstants.SUPPLY_CURRENT = 30.0  # Amperes
            Constants.LauncherConstants.MOMENT_OF_INERTIA =  0.0030700826
            Constants.LauncherConstants.FLYWHEEL_RADIUS = 2.0 * 0.0254
            #Constants.LauncherConstants.FEED_FORWARD = 4.0
            Constants.LauncherConstants.MAX_RPS = 75.0

            # Feeder
            Constants.FeederConstants.GEAR_RATIO = 1.0  # Adjust based on actual gear ratio
            Constants.FeederConstants.GAINS = (Slot0Configs()
                .with_k_p(0.3)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
            Constants.FeederConstants.SUPPLY_CURRENT = 30.0  # Amperes
            Constants.FeederConstants.MOMENT_OF_INERTIA = 0.0067
            Constants.FeederConstants.FEED_FORWARD = 3.0

# Initialize hardware configs at module load time
_init_hardware_configs()
