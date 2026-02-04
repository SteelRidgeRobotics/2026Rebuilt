"""StateSubsystem class for simple state subsystems."""
from enum import Enum, EnumMeta

from commands2 import Command
from commands2.subsystem import Subsystem
from pykit.logger import Logger


class StateSubsystem(Subsystem):
    """
    Subsystem class for handling subsystem state transitions.
    Updated for 2026 to utilize PyKit.
    """

    class SubsystemState(Enum):
        """
        Possible state subsystem states.
        Subclasses should extend this enum.
        """
        OVERRIDE_THIS_ENUM = None

    def __init__(self, name: str, starting_state: SubsystemState):
        """
        Sets the default state of the subsystem to starting_state.

        Last state is also set to starting state, meaning `self.has_changed()` will not
        trigger on __init__.

        :param name: Name of the subsystem
        :type name: str
        :param starting_state: Starting state of the subsystem
        :type starting_state: SubsystemState
        """
        super().__init__()
        self.setName(name.title())
        self._starting_state = starting_state

        if not hasattr(self.__class__, "SubsystemState"):
            # Must have a SubsystemState
            raise TypeError("Subsystems must define a SubsystemState.")

        state_enum = self.__class__.SubsystemState
        if not isinstance(state_enum, EnumMeta):
            # SubsystemState needs to be some sort of Enum
            raise TypeError(
                f"{self.__class__.__name__}.SubsystemState must be an Enum."
            )

        if "OVERRIDE_THIS_ENUM" in state_enum.__members__:
            # Subclasses must implement their own SubsystemState
            raise TypeError("Subsystems must override SubsystemState.")

        self._locked = False
        self._last_state = self._subsystem_state = starting_state
        self._has_changed = False

    def periodic(self):
        # Ensure the state is correctly set on startup

        if self._last_state != self._subsystem_state:
            self._has_changed = True
        else:
            self._has_changed = False
        self._last_state = self._subsystem_state

        # PyKit logging
        Logger.recordOutput(
            f"{self.getName()}/Subsystem State",
            self._subsystem_state.name
        )
        Logger.recordOutput(f"{self.getName()}/Has Changed", self._has_changed)
        Logger.recordOutput(f"{self.getName()}/Locked", self._locked)
        Logger.recordOutput(f"{self.getName()}/Last State", self._last_state.name)

    def lock_state(self) -> None:
        """Prevents state changes."""
        self._locked = True

    def unlock_state(self) -> None:
        """Allows state changes."""
        self._locked = False

    def is_locked(self) -> bool:
        """Returns True if the subsystem is locked."""
        return self._locked

    def is_unlocked(self) -> bool:
        """Returns True if the subsystem is not locked."""
        return not self._locked

    def has_state_changed(self) -> bool:
        """Returns True if the subsystem state has changed since the last loop."""
        return self._has_changed

    def on_state_change(self, old: SubsystemState, new: SubsystemState) -> None:
        """Called when the state changes. Override if needed."""

    def get_last_state(self) -> SubsystemState:
        """Returns the last state of the subsystem."""
        return self._last_state

    def get_current_state(self) -> SubsystemState | None:
        """Returns the current state of the subsystem."""
        return self._subsystem_state

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        """Sets the desired state of the subsystem."""
        if not self.is_locked() and desired_state != self._subsystem_state:
            self._subsystem_state = desired_state
            self.on_state_change(self._last_state, desired_state)

    def set_desired_state_command(self, state: SubsystemState) -> Command:
        """Sets the desired state of the subsystem with an InstantCommand."""
        return self.runOnce(lambda: self.set_desired_state(state))
