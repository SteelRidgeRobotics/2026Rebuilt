"""Main robot file"""
import os

import wpilib
from commands2 import CommandScheduler, Command
from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger
from pykit.logtable import LogTable
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.wpilog.wpilogwriter import WPILOGWriter
from wpilib import DriverStation, Timer

import util
from constants import Constants
from lib import elasticlib
from lib.elasticlib import Notification, NotificationLevel
from robot_config import currentRobot, has_subsystem
from robot_container import RobotContainer

# Prevent int64 overflows in PyKit (apparently that exists).
_INT64_MAX = 2 ** 63 - 1
_original_get_timestamp = LogTable.getTimestamp


def _patched_get_timestamp(self):
    ts = _original_get_timestamp(self)
    if ts > _INT64_MAX:
        ts = ts // 1000  # Assume nanoseconds -> microseconds
    ts = min(ts, _INT64_MAX)
    return ts


LogTable.getTimestamp = _patched_get_timestamp


class Dwayne(LoggedRobot):
    """Dwayne: The robot, the myth, the legend, and the baller."""

    def __init__(self) -> None:
        # Suppress WPILib loop-overrun/watchdog messages so they don't flood logs
        util.install_loop_overrun_stderr_filter()
        super().__init__()

        Logger.recordMetadata("Robot", currentRobot.name)
        Logger.recordMetadata("Mode", Constants.CURRENT_MODE.name)

        # Record subsystem metadata
        Logger.recordMetadata("ClimberPresent", str(has_subsystem("climber")))
        Logger.recordMetadata("FeederPresent", str(has_subsystem("feeder")))
        Logger.recordMetadata("HoodPresent", str(has_subsystem("hood")))
        Logger.recordMetadata("IntakePresent", str(has_subsystem("intake")))
        Logger.recordMetadata("LauncherPresent", str(has_subsystem("intake")))
        Logger.recordMetadata("VisionPresent", "True")
        Logger.recordMetadata("DrivetrainPresent", "True")

        match Constants.CURRENT_MODE:
            # Running on a real robot, log to a USB stick ("/U/logs")
            case Constants.Mode.REAL:
                deploy_config = wpilib.deployinfo.getDeployData()
                if deploy_config is not None:
                    Logger.recordMetadata(
                        "Deploy Host", deploy_config.get("deploy-host", "")
                    )
                    Logger.recordMetadata(
                        "Deploy User", deploy_config.get("deploy-user", "")
                    )
                    Logger.recordMetadata(
                        "Deploy Date", deploy_config.get("deploy-date", "")
                    )
                    Logger.recordMetadata(
                        "Code Path", deploy_config.get("code-path", "")
                    )
                    Logger.recordMetadata(
                        "Git Hash",
                        deploy_config.get("git-hash", "")
                    )
                    Logger.recordMetadata(
                        "Git Branch", deploy_config.get("git-branch", "")
                    )
                    Logger.recordMetadata(
                        "Git Description", deploy_config.get("git-desc", "")
                    )
                Logger.addDataReciever(WPILOGWriter())
                Logger.addDataReciever(NT4Publisher(True))

            # Running a physics simulator, log to NT
            case Constants.Mode.SIM:
                Logger.addDataReciever(NT4Publisher(True))
                #Logger.addDataReciever(WPILOGWriter())

            # Replaying a log, set up replay source
            case Constants.Mode.REPLAY:
                log_path = os.environ["LOG_PATH"]
                log_path = os.path.abspath(log_path)
                print(f"Starting log from {log_path}")
                Logger.setReplaySource(WPILOGReader(log_path))
                Logger.addDataReciever(
                    WPILOGWriter(log_path[:-7] + "_sim.wpilog")
                )

        # Avoid CAN errors from pykit when no PDH/PDP is on the bus (or
        # wrong module ID)
        util.install_safe_power_distribution_logging()

        # Start PyKit logger
        Logger.start()

        DriverStation.silenceJoystickConnectionWarning(
            not DriverStation.isFMSAttached()
        )
        self.container = RobotContainer()

        # Disable Phoenix 6's signal logging (no need if we're using PyKit)
        SignalLogger.enable_auto_logging(False)
        SignalLogger.stop()
        wpilib.LiveWindow.disableAllTelemetry()  # leg

        # We can check ourselves if we're overrunning
        # And the warning it prints is laggy, so this makes it only tell us if
        # We're REALLY lagging
        CommandScheduler.getInstance().setPeriod(1.0)

        dashboard_nt = NetworkTableInstance.getDefault().getTable("Elastic")
        self._match_time_pub = dashboard_nt.getFloatTopic(
            "Match Time"
        ).publish()
        self._game_phase_name_pub = dashboard_nt.getStringTopic(
            "Game Phase"
        ).publish()
        self._game_phase_time_pub = dashboard_nt.getFloatTopic(
            "Phase Time"
        ).publish()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()
        self._match_time_pub.set(Timer.getMatchTime())
        phase_name, phase_time = util.get_game_phase()
        self._game_phase_name_pub.set(phase_name)
        self._game_phase_time_pub.set(float(phase_time))
        if self.container.drivetrain is not None:
            # pylint: disable=protected-access
            self.container._field.setRobotPose(
                self.container.drivetrain.get_cached_state().pose
            )

        Logger.recordOutput("Components", self.container.get_component_poses())

    def _simulationPeriodic(self) -> None:
        self.container.fuel_sim.update_sim()

    def autonomousInit(self) -> None:
        selected_auto = self.container.get_autonomous_command()
        if selected_auto is not None:
            print(f"Selected Auto: {selected_auto.getName()}")
            selected_auto.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def autonomousExit(self) -> None:
        print("Autonomous period ended")

    def teleopInit(self) -> None:
        print("Teleoperated period started")
        command = self.container.get_autonomous_command()
        if isinstance(command, Command):
            command.cancel()

    def teleopExit(self) -> None:
        print("Teleoperated period ended")
        if DriverStation.isFMSAttached():
            elasticlib.send_notification(
                Notification(
                    level=NotificationLevel.INFO.value,
                    title="Good match!",
                    description="(again)" if DriverStation.getReplayNumber()
                                             > 1 else ""
                )
            )

    def testInit(self):
        print("Test period started")
        CommandScheduler.getInstance().cancelAll()

    def disabledInit(self):
        if self.container.vision is not None:
            self.container.vision.set_throttle(150)

    def disabledExit(self):
        if self.container.vision is not None:
            self.container.vision.set_throttle(0)

    def testExit(self):
        print("Test period ended")

    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass
