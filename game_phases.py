import wpilib
from wpilib import DriverStation

def get_phase():
    # 1. Goes to get data from DriverStation
    # getMatchTime() returns seconds left in the current period
    time = DriverStation.getMatchTime()

    # 2. Goes to get game-specific data like "L", "R", "RRR"
    game_data = DriverStation.getGameSpecificMessage()

    # 3. Check if the match has actually started or if the data is valid
    # In FRC, time is generally None or -1 before the match starts
    if time is None or time <= 0:
        return "PRE-MATCH", game_data

# Using match-case for nicer state handling
# We match against a tuple of (isAutonomous, isTeleop, time)
match (DriverStation.isAutonomous(), DriverStation.isTeleop(), time):

    case (True, _, _):
        return "Auto", game_data
    
    case (_, True, t) if t <= 20:
        return "ENDGAME", game_data

    case (_, True, _):
        return "TELEOP", game_data

    case _:
        return "DISABLED", game_data
    
    # 4. FRC Match Timing reasoning
    # During Autonomous, the timer begins at 15.0 and counts down to 0.
    if DriverStation.isAutonomous():
        return "AUTO", game_data

    # During Teleop, the timer starts at 135.0 and goes until it reaches 0.
    if DriverStation.isTeleop():
        # Inspect if we are in the final 30 seconds (Endgame)
        if time <= 30:
            return "ENDGAME", game_data
        else:
            return "TELEOP", game_data

    # Retreat for Disabled or Test mode
    return "DISABLED", game_data

    def teleopPeriodic(self):
        phase, data = get_phase()

        match phase:
            case "ENDGAME":
                self.operator_controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 1.0)
            case "TELEOP":
                # Common Teleop driving logic here
                pass
