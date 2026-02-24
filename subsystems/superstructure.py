from enum import auto, Enum
from typing import Optional

from commands2 import Command, Subsystem, cmd
from wpilib import DriverStation

from subsystems.intake import IntakeSubsystem
from subsystems.feeder import FeederSubsystem
from subsystems.launcher import LauncherSubsystem
from subsystems.hood import HoodSubsystem
from subsystems.turret import TurretSubsystem


class Superstructure(Subsystem):
    """
    The Superstructure is in charge of handling all subsystems to ensure no conflicts between them.
    """

    class Goal(Enum):
        DEFAULT     = auto()  # Default goal
        INTAKE      = auto()  # Intaking fuel from the floor.  This goal may be removed
        LAUNCH      = auto()  # Scoring fuel into the hub
        AIMHUB      = auto()  # Point turret to hub
        AIMOUTPOST  = auto()  # Point turret to the outpost side of the alliance center
        AIMDEPOT    = auto()  # Point turret to the depot side of the alliance center

    # Map each goal to each subsystem state to reduce code complexity
    _goal_to_states: dict[Goal,
        tuple[
            Optional[IntakeSubsystem.SubsystemState],
            Optional[FeederSubsystem.SubsystemState],
            Optional[LauncherSubsystem.SubsystemState],
            Optional[HoodSubsystem.SubsystemState],
            Optional[TurretSubsystem.SubsystemState],
        ]] = {

        Goal.DEFAULT : (
            IntakeSubsystem.SubsystemState.STOP,
            FeederSubsystem.SubsystemState.STOP,
            LauncherSubsystem.SubsystemState.IDLE,
            HoodSubsystem.SubsystemState.STOW,
            TurretSubsystem.SubsystemState.HUB,
        ),

        Goal.INTAKE : (
            IntakeSubsystem.SubsystemState.INTAKE,
            FeederSubsystem.SubsystemState.STOP,
            LauncherSubsystem.SubsystemState.IDLE,
            HoodSubsystem.SubsystemState.STOW,
            None
        ),

        Goal.LAUNCH : (
            IntakeSubsystem.SubsystemState.INTAKE,
            FeederSubsystem.SubsystemState.INWARD,
            LauncherSubsystem.SubsystemState.SCORE,
            None, None
        ),

        Goal.AIMHUB : (
            None, None, None, 
            HoodSubsystem.SubsystemState.AIMBOT, 
            TurretSubsystem.SubsystemState.HUB
        ),

        Goal.AIMOUTPOST : (
            None, None, None, 
            HoodSubsystem.SubsystemState.PASS, 
            TurretSubsystem.SubsystemState.OUTPOST
        ),

        Goal.AIMDEPOT : (
            None, None, None, 
            HoodSubsystem.SubsystemState.PASS, 
            TurretSubsystem.SubsystemState.DEPOT
        ),

    }

    def __init__(self, 
            intake: Optional[IntakeSubsystem] = None,
            feeder: Optional[FeederSubsystem] = None,
            launcher: Optional[LauncherSubsystem] = None,
            hood: Optional[HoodSubsystem] = None,
            turret: Optional[TurretSubsystem] = None
                ) -> None:
        """
        Constructs the superstructure using instance of each subsystem.
        Subsystems are optional to support robots that don't have all hardware.

        :param intake: Subsystem that handles the intake
          :type intake: Optional[IntakeSubsystem]
        :param feeder: Subsystem that handles the feeder
          :type feeder: Optional[FeederSubsystem]
        :param launcher: Subsystem that handles the launcher
          :type launcher: Optional[LauncherSubsystem]
        :param hood: Subsystem that handles the hood
          :type hood: Optional[HoodSubsystem]
        :param turret: Subsystem that handles the turret
          :type turret: Optional[TurretSubsystem]
        """
        super().__init__()

        self.intake = intake
        self.feeder = feeder
        self.launcher = launcher
        self.hood = hood
        self.turret = turret

        self._goal = self.Goal.DEFAULT
        self.set_goal_command(self._goal)

        # table = NetworkTableInstance.getDefault().getTable("Superstructure")
        # self._current_goal_pub = table.getStringTopic("Current Goal").publish()
        # self._component_poses = table.getStructArrayTopic("Components", Pose3d).publish()

    def periodic(self):
        if DriverStation.isDisabled():
            return
        # TODO add other subsystem periodic functions

    def _set_goal(self, goal: Goal) -> None:
        self._goal = goal

        intake_state, feeder_state, launcher_state, hood_state, turret_state = self._goal_to_states.get(goal, (None, None, None, None, None))

        if not (self.intake is None or intake_state is None):
            self.intake.set_desired_state(intake_state)

        if not (self.feeder is None or feeder_state is None):
            self.feeder.set_desired_state(feeder_state)

        if not (self.launcher is None or launcher_state is None):
            self.launcher.set_desired_state(launcher_state)

        if not (self.hood is None or hood_state is None):
            self.hood.set_desired_state(hood_state)

        if not (self.turret is None or turret_state is None):
            self.turret.set_desired_state(turret_state)

    def set_goal_command(self, goal: Goal) -> Command:
        """
        Return a command that sets the superstructure goal to whatever the desired goal is.

        :param goal: The desired goal
        :type goal:  Goal
        :return:     A command that will set the desired goal
        :rtype:      Command
        """
        return cmd.runOnce(lambda: self._set_goal(goal), self)
