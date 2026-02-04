"""
Provides the StateSubsystem base class for simple discrete state subsystems.

Subclasses should override `SubsystemState` enum with their own states.
"""
from enum import Enum

from commands2 import Command
from commands2.subsystem import Subsystem
from pykit.logger import Logger


class StateSubsystem(Subsystem):
    """Base class for subsystems that operate with distinct states."""

    class SubsystemState(Enum):
        """
        Possible state subsystem states.
        Subclasses should extend this enum.
        """
        OVERRIDE_THIS_ENUM = None

    def __init__(self, name: str, starting_state: SubsystemState):
        """
        Sets the default state of the subsystem to starting_state.

        :param name: Name of the subsystem
        :type name: str
        :param starting_state: Starting state of the subsystem
        :type starting_state: SubsystemState
        """
        super().__init__()
        self.setName(name.title())

        if "OVERRIDE_THIS_ENUM" in self.__class__.SubsystemState.__members__:
            # Subclasses must implement their own SubsystemState
            raise TypeError("Subsystems must override SubsystemState.")

        self._locked = False
        self._last_state = self._current_state = starting_state

        self._log_state()

    def periodic(self):
        """
        Update PyKit logs and `self._last_state`.

        If you plan to use `self.state_changed`, call super().periodic() at the
        end of your function.
        """
        if self.state_changed:
            self._log_state()

        Logger.recordOutput(
            f"{self.getName()}/Has Changed",
            self.state_changed
        )

        self._last_state = self._current_state

    @property
    def is_locked(self) -> bool:
        """Returns True if the subsystem is locked."""
        return self._locked

    @is_locked.setter
    def is_locked(self, lock: bool) -> None:
        self._locked = lock
        Logger.recordOutput(f"{self.getName()}/Locked", lock)

    def lock(self):
        """Locks the subsystem from switching states."""
        self.is_locked = True

    def unlock(self):
        """Unlocks the subsystem, allowing switching states."""
        self.is_locked = False

    @property
    def current_state(self) -> SubsystemState:
        """Returns the current state of the subsystem."""
        return self._current_state

    @property
    def last_state(self) -> SubsystemState:
        """Returns the last state of the subsystem."""
        return self._last_state

    @property
    def state_changed(self) -> bool:
        """Returns True if the subsystem has changed since the last periodic loop."""
        return self._last_state != self._current_state

    def on_state_change(self, old: SubsystemState, new: SubsystemState) -> None:
        """Called when the state changes. Override if needed."""

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        """Sets the desired state of the subsystem."""
        if not self._locked and desired_state != self._current_state:
            old_state = self._current_state
            self._current_state = desired_state
            self.on_state_change(old_state, desired_state)

    def set_desired_state_command(self, state: SubsystemState) -> Command:
        """Sets the desired state of the subsystem with an InstantCommand."""
        return self.runOnce(lambda: self.set_desired_state(state))

    def _log_state(self) -> None:
        Logger.recordOutput(
            f"{self.getName()}/Subsystem State",
            self._current_state.name
        )
