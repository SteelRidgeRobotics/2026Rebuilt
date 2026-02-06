"""
Provides the StateSubsystem base class for simple discrete state subsystems.

New feature list for 2026:
- Freeze/Unfreeze -> Lock/Unlock (better readability)
- PyKit logging
- Improved state transitioning (`on_state_change` and `self.last_state`)
- Minor type checking
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
        Subclasses need to override this enum.

        All states should be unique and have different values.
        """

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

        self._locked = False

        if not isinstance(starting_state, self.SubsystemState):
            raise TypeError("starting_state must be a SubsystemState")
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
        """
        Returns True if the subsystem is locked.

        Locked subsystems can't change states.
        """
        return self._locked

    @is_locked.setter
    def is_locked(self, lock: bool) -> None:
        self._locked = lock
        Logger.recordOutput(f"{self.getName()}/Locked", lock)

    def lock(self):
        """Locks the subsystem from switching states."""
        self.is_locked = True

    def unlock(self):
        """Unlocks the subsystem, allowing new state changes."""
        self.is_locked = False

    @property
    def current_state(self) -> SubsystemState:
        """Returns the current state of the subsystem."""
        return self._current_state

    @property
    def last_state(self) -> SubsystemState:
        """Returns the last state of the subsystem (from the last periodic call)."""
        return self._last_state

    @property
    def state_changed(self) -> bool:
        """Returns True if the subsystem has changed since the last periodic loop."""
        return self._last_state != self._current_state

    def on_state_change(self, old: SubsystemState, new: SubsystemState) -> None:
        """Called when the state changes. Override if needed."""

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        """
        Sets the desired state of the subsystem.

        IT IS REQUIRED TO CALL `super().set_desired_state()` TO MODIFY STATES.
        DO NOT IMPLEMENT YOUR OWN.
        """
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
