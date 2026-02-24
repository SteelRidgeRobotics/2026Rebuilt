import wpilib
from commands2 import CommandScheduler
from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger
from pykit.logreplaysource import LogReplaySource
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.wpilog.wpilogwriter import WPILOGWriter
from wpilib import DataLogManager, DriverStation, Timer

import robot_config

# Workaround for PyKit: ntcore IntegerPublisher and wpiutil DataLog expect int64,
# but LogTable.getTimestamp() can exceed int64 max. Patch at source to fix all
# receivers (NT4Publisher, WPILOGWriter, etc.).
_INT64_MAX = 2**63 - 1
from pykit.logtable import LogTable
_original_get_timestamp = LogTable.getTimestamp
def _patched_get_timestamp(self):
    ts = _original_get_timestamp(self)
    if ts > _INT64_MAX:
        ts = ts // 1000  # Assume nanoseconds -> microseconds
    if ts > _INT64_MAX:
        ts = _INT64_MAX
    return ts
LogTable.getTimestamp = _patched_get_timestamp
from constants import Constants
from lib import elasticlib
from lib.elasticlib import Notification, NotificationLevel
from robot_container import RobotContainer
import util


class Dwayne(LoggedRobot):

    def __init__(self, period = 0.02) -> None:
        super().__init__()

        Logger.recordMetadata("Robot", robot_config.currentRobot.name.title())

        match Constants.currentMode:
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
                    Logger.recordMetadata("Git Hash", deploy_config.get("git-hash", ""))
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

            # Replaying a log, set up replay source
            case Constants.Mode.REPLAY:
                self.useTiming = False
                Logger.setReplaySource(LogReplaySource())
                Logger.addDataReciever(WPILOGWriter(None))

        # Avoid CAN errors from pykit when no PDH/PDP is on the bus (or wrong module ID)
        util._install_safe_power_distribution_logging()

        # Start PyKit logger
        Logger.start()

        DriverStation.silenceJoystickConnectionWarning(not DriverStation.isFMSAttached())
        self.container = RobotContainer()

        SignalLogger.enable_auto_logging(False)
        wpilib.LiveWindow.disableAllTelemetry()

        DataLogManager.log("Robot initialized")

        dashboard_nt = NetworkTableInstance.getDefault().getTable("Elastic")
        self._match_time_pub = dashboard_nt.getFloatTopic("Match Time").publish()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()
        self._match_time_pub.set(Timer.getMatchTime())
        if self.container.drivetrain is not None:
            self.container._field.setRobotPose(self.container.drivetrain.get_state().pose)

    def _simulationPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        selected_auto = self.container.get_autonomous_command()
        if selected_auto is not None:
            DataLogManager.log(f"Selected Auto: {selected_auto.getName()}")
            selected_auto.schedule()

        elasticlib.select_tab("Autonomous")
            
    def autonomousPeriodic(self) -> None:
        pass
    
    def autonomousExit(self) -> None:
        DataLogManager.log("Autonomous period ended")
        elasticlib.select_tab("Teleop")
            
    def teleopInit(self) -> None:
        DataLogManager.log("Teleoperated period started")
        self.container.get_autonomous_command().cancel()

    def teleopExit(self) -> None:
        DataLogManager.log("Teleoperated period ended")
        if DriverStation.isFMSAttached():
            elasticlib.send_notification(
                Notification(
                    level=NotificationLevel.INFO.value,
                    title="Good match!",
                    description="(again)" if DriverStation.getReplayNumber() > 1 else ""
                )
            )

    def testInit(self):
        DataLogManager.log("Test period started")
        CommandScheduler.getInstance().cancelAll()
        elasticlib.select_tab("Debug")
        SignalLogger.start()

    def disabledInit(self):
        if self.container.vision is not None:
            self.container.vision.set_throttle(150)

    def disabledExit(self):
        if self.container.vision is not None:
            self.container.vision.set_throttle(0)

    def testExit(self):
        pass
    
    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass