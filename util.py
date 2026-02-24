import threading
import time
from abc import ABC
from collections import deque
from typing import Final, Callable, List, Deque

from ntcore import DoubleEntry, NetworkTableInstance
from phoenix6 import StatusCode, BaseStatusSignal, CANBus
from pykit.logger import Logger
from pykit.logtable import LogTable
from pykit.networktables.loggednetworkinput import LoggedNetworkInput
from wpilib import Timer, RobotController, PowerDistribution
from pykit.inputs.loggablepowerdistribution import LoggedPowerDistribution

from constants import Constants


class PhoenixOdometryThread(threading.Thread):
    _instance = None
    _instance_lock = threading.Lock()

    odometryLock: Final[threading.Lock] = threading.Lock()

    @staticmethod
    def getInstance():
        with PhoenixOdometryThread._instance_lock:
            if PhoenixOdometryThread._instance is None:
                PhoenixOdometryThread._instance = PhoenixOdometryThread()
            return PhoenixOdometryThread._instance

    def __init__(self):
        super().__init__(name="PhoenixOdometryThread", daemon=True)

        self.signals_lock = threading.Lock()

        self.phoenix_signals: List[BaseStatusSignal] = []
        self.generic_signals: List[Callable[[], float]] = []

        self.phoenix_queues: List[Deque[float]] = []
        self.generic_queues: List[Deque[float]] = []
        self.timestamp_queues: List[Deque[float]] = []

        self.is_can_fd = CANBus("*").is_network_fd()

    # Only start thread if timestamps are actually requested
    def start(self):
        if len(self.timestamp_queues) > 0:
            super().start()

    # Phoenix signal registration
    def registerSignal(self, signal: BaseStatusSignal) -> Deque[float]:
        queue = deque(maxlen=20)

        with self.signals_lock:
            with PhoenixOdometryThread.odometryLock:
                self.phoenix_signals.append(signal)
                self.phoenix_queues.append(queue)

        return queue

    # Generic (lambda / callable) signal registration
    def registerGenericSignal(self, supplier: Callable[[], float]) -> Deque[float]:
        queue = deque(maxlen=20)

        with self.signals_lock:
            with PhoenixOdometryThread.odometryLock:
                self.generic_signals.append(supplier)
                self.generic_queues.append(queue)

        return queue

    # Timestamp queue
    def makeTimestampQueue(self) -> Deque[float]:
        queue = deque(maxlen=20)

        with PhoenixOdometryThread.odometryLock:
            self.timestamp_queues.append(queue)

        return queue

    def run(self):
        while True:
            # Wait for updates
            with self.signals_lock:
                try:
                    if self.is_can_fd and len(self.phoenix_signals) > 0:
                        BaseStatusSignal.wait_for_all(
                            2.0 / 250.0,
                            *self.phoenix_signals
                        )
                    else:
                        # Non-CAN FD fallback
                        time.sleep(1.0 / 100.0)
                        if len(self.phoenix_signals) > 0:
                            BaseStatusSignal.refresh_all(*self.phoenix_signals)

                except Exception as e:
                    print(e)

            # Push samples
            with PhoenixOdometryThread.odometryLock:
                # FPGA timestamp (seconds)
                timestamp = RobotController.getFPGATime() / 1e6

                total_latency = 0.0
                for sig in self.phoenix_signals:
                    total_latency += sig.timestamp.get_latency()

                if len(self.phoenix_signals) > 0:
                    timestamp -= total_latency / len(self.phoenix_signals)

                # Phoenix signals
                for sig, queue in zip(self.phoenix_signals, self.phoenix_queues):
                    queue.append(sig.value_as_double)

                # Generic signals
                for supplier, queue in zip(self.generic_signals, self.generic_queues):
                    queue.append(supplier())

                # Timestamps
                for queue in self.timestamp_queues:
                    queue.append(timestamp)


def tryUntilOk(maxAttempts: int, command: Callable[[], StatusCode]) -> None:
    """Attempts to run the command until no error is produced."""
    for i in range(maxAttempts):
        error = command()
        if error.is_ok():
            break

class LoggableInputs(ABC):

    def toLog(self, table: LogTable) -> None:
        pass

    def fromLog(self, table: LogTable) -> None:
        pass

class LoggedNetworkNumber(LoggedNetworkInput):

    def __init__(self, key: str, defaultValue: float | None = None) -> None:
        super().__init__()
        self._key = key
        self._entry: DoubleEntry = NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(0.0)
        self._defaultValue = 0.0
        self._value = self._defaultValue
        Logger.registerDashboardInput(self)

        if defaultValue is not None:
            self.setDefault(defaultValue)
            self._value = defaultValue

        outer = self

        class LoggedNetworkNumberInputs(LoggableInputs):
            def toLog(self, table: LogTable, prefix: str) -> None:
                table.put(
                    LoggedNetworkInput.removeSlash(outer._key),
                    outer._value
                )

            def fromLog(self, table: LogTable, prefix: str) -> None:
                outer._value = table.get(
                    LoggedNetworkInput.removeSlash(outer._key),
                    outer._defaultValue
                )

        self._inputs: Final[LoggableInputs] = LoggedNetworkNumberInputs()

    def setDefault(self, defaultValue: float) -> None:
        self._defaultValue = defaultValue
        self._entry.set(self._entry.get(self._defaultValue))

    def set(self, value: float) -> None:
        self._entry.set(value)

    def get(self) -> float:
        return self._value

    def periodic(self):
        if not Logger.replaySource:
            self._value = self._entry.get(self._defaultValue)
        Logger.processInputs(super().prefix, self._inputs)


class LoggedTunableNumber:
    """
    Class for a tunable number.
    Gets value from dashboard in tuning mode, returns default if not or value not in dashboard.
    """
    _tableKey = "/Tuning"

    def __init__(self, dashboardKey: str, defaultValue: float|None=None) -> None:
        self._defaultValue = None
        self._dashboardNumber = None
        self._key = f"{self._tableKey}/{dashboardKey}"
        self._lastHasChangedValues = {}
        self._hasDefault = False
        if defaultValue is not None:
            self.initDefault(defaultValue)

    def initDefault(self, defaultValue: float) -> None:
        if not self._hasDefault:
            self._hasDefault = True
            self._defaultValue = defaultValue
            if Constants.tuningMode:
                self._dashboardNumber = LoggedNetworkNumber(self._key, defaultValue)

    def get(self) -> float:
        if not self._hasDefault:
            return 0.0
        return self._dashboardNumber.get() if Constants.tuningMode else self._defaultValue

    def hasChanged(self, some_id: int) -> bool:
        currentValue = self.get()
        lastValue = self._lastHasChangedValues.get(some_id)
        if lastValue is None or currentValue != lastValue:
            self._lastHasChangedValues[some_id] = currentValue
            return True
        return False

    @staticmethod
    def ifChanged(some_id: int, action: Callable[[List[float]], None]|Callable[[], None], *tunableNumbers: "LoggedTunableNumber") -> None:
        if any(tunableNumber.hasChanged(some_id) for tunableNumber in tunableNumbers):
            try:
                action([tunableNumber.get() for tunableNumber in tunableNumbers])
            except TypeError:
                action()


class LoggedTracer:
    _startTime = -1

    @classmethod
    def reset(cls) -> None:
        cls._startTime = Timer.getFPGATimestamp()

    @classmethod
    def record(cls, epochName: str) -> None:
        now = Timer.getFPGATimestamp()
        print(f"LoggedTracer/{epochName}MS", (now - cls._startTime) * 1000)
        #Logger.recordOutput(f"LoggedTracer/{epochName}MS", (now - cls._startTime) * 1000)
        cls._startTime = now

def _install_safe_power_distribution_logging() -> None:
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