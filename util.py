"""Miscellaneous functions used throughout the robot."""
import sys
from typing import Callable, Optional

from phoenix6 import StatusCode
from pykit.inputs.loggablepowerdistribution import LoggedPowerDistribution
from wpilib import PowerDistribution, DriverStation
from wpimath.geometry import Pose2d, Translation2d

from constants import Constants

# Substrings that identify WPILib loop-overrun/watchdog/epoch messages to suppress from stderr
_LOOP_OVERRUN_SUPPRESS_PATTERNS = (
    "overrun",
    "Watchdog not fed",
    "PrintEpochs",
    "PrintLoopOverrunMessage",
    "SmartDashboard::UpdateValues()",
    "Shuffleboard::Update()",
    "RobotPeriodic():",
    "LiveWindow::UpdateValues()",
    "AutonomousPeriodic():",
    "TeleopPeriodic():",
    "DisabledPeriodic():",
    "TestPeriodic():",
)


def install_loop_overrun_stderr_filter() -> None:
    """
    Filter WPILib loop-overrun and watchdog messages from stderr so they don't
    clutter logs. Only lines matching known timing/overrun patterns are
    suppressed; real errors are still printed.
    """
    _real_stderr = sys.stderr

    class _FilteredStderr:
        def write(self, text: str) -> int:
            if not text.strip():
                return _real_stderr.write(text)
            for pattern in _LOOP_OVERRUN_SUPPRESS_PATTERNS:
                if pattern in text:
                    return len(text)  # suppress: pretend we wrote it
            return _real_stderr.write(text)

        def flush(self) -> None:
            _real_stderr.flush()

        def __getattr__(self, name: str):
            return getattr(_real_stderr, name)

    sys.stderr = _FilteredStderr()


PHASE_BOUNDARIES = [
    (130, "S1", False),
    (105, "S2", True),
    (80,  "S3", False),
    (55,  "S4", True),
    (30,  "E",  None),  # None = endgame, no hub status
]

def get_game_phase() -> tuple[str, int]:
    """Returns the current game phase and remaining time in that phase."""
    game_message = DriverStation.getGameSpecificMessage() # Returns the winning alliance for auto as either R or B
 
    match_time = int(DriverStation.getMatchTime())
    if DriverStation.isAutonomous():
        return "Autonomous", match_time
    if not DriverStation.isTeleop():
        return "Disabled", match_time

    is_blue = DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    match match_time:
        case x if x <= 30.0:
            return "E (Active)", match_time
        case x if 30.0 < x <= 55.0:
            status = hub_status(True, game_message, is_blue)
            return f"S4 ({status})", match_time - 30.0
        case x if 55.0 < x <= 80.0:
            status = hub_status(False, game_message, is_blue)
            return f"S3 ({status})", match_time - 55.0
        case x if 80.0 < x <= 105.0:
            status = hub_status(True, game_message, is_blue)
            return f"S2 ({status})", match_time - 80.0
        case x if 105.0 < x <= 130.0:
            status = hub_status(False, game_message, is_blue)
            return f"S1 ({status})", match_time - 105.0
        case _:
            return "T (Active)", match_time - 130.0

def hub_status(winner_active, game_message, is_blue):
    """
    Returns the status of the hub based on the winner active and winner.
    """
    if not game_message:
        return "Unknown"

    if winner_active and game_message == "B" and is_blue:
        return "Active"
    elif winner_active and game_message == "R" and not is_blue:
        return "Active"
    elif not winner_active and game_message == "B" and not is_blue:
        return "Active"
    elif not winner_active and game_message == "R" and is_blue:
        return "Active"
    else:
        return "Inactive"

def try_until_ok(max_attempts: int, command: Callable[[], StatusCode]) -> None:
    """Attempts to run the command until no error is produced."""
    for _ in range(max_attempts):
        error = command()
        if error.is_ok():
            break

def install_safe_power_distribution_logging() -> None:
    """
    Install a Power Distribution logger that does not spam CAN errors when no PDH/PDP
    is present. PyKit's Logger calls LoggedPowerDistribution.getInstance().saveToTable()
    every cycle; if no device exists at the expected CAN ID, that causes repeated
    "CAN: Message not found: Module N" errors.
    """
    module_id = getattr(
        Constants.CanIDs, "POWER_DISTRIBUTION_MODULE_ID", None
    )
    if module_id is not None:
        # PDH/PDP present: use real device but catch CAN errors so one bad cycle doesn't spam
        class _SafeLoggedPowerDistribution(LoggedPowerDistribution):
            def saveToTable(self, table):
                try:
                    super().saveToTable(table)
                except Exception:
                    table.put("Voltage", 0.0)
                    table.put("TotalCurrent", 0.0)
                    table.put("TotalPower", 0.0)
                    table.put("TotalEnergy", 0.0)
                    table.put("Temperature", 0.0)
                    table.put("ChannelCurrentsList", [])
                    table.put("ChannelCurrentsTotal", 0.0)

        LoggedPowerDistribution.instance = _SafeLoggedPowerDistribution(
            moduleId=module_id,
            moduleType=PowerDistribution.ModuleType.kRev,
        )
    else:
        # No PDH: stub that never touches CAN
        class _StubLoggedPowerDistribution(LoggedPowerDistribution):
            def __init__(self) -> None:
                self.moduleId = 0
                self.moduleType = PowerDistribution.ModuleType.kRev
                self.distribution = None

            def saveToTable(self, table):
                table.put("Voltage", 0.0)
                table.put("TotalCurrent", 0.0)
                table.put("TotalPower", 0.0)
                table.put("TotalEnergy", 0.0)
                table.put("Temperature", 0.0)
                table.put("ChannelCurrentsList", [])
                table.put("ChannelCurrentsTotal", 0.0)

        LoggedPowerDistribution.instance = _StubLoggedPowerDistribution()
