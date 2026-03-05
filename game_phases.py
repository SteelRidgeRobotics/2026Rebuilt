import wpilib
from wpilib import DriverStation

def get_phase():
    time = DriverStation.getMatchTime()
    game_data = DriverStation.getGameSpecificMessage()
   
    if time is None or time <= 0:
        return "PRE-MATCH", game_data
   
    if DriverStation.isAutonomous():
        return "AUTO", game_data
    elif DriverStation.isTeleop():
        if time <= 30:
            return "ENDGAME", game_data
        else:
            return "TELEOP", game_data
    return "DISABLED", game_data

class GamePhasesClass:  # This adds everything correctly so self works
    def teleopPeriodic(self):
        phase, data = get_phase()
        match phase:
            case "AUTO":  # This manages the autonomous period when get_phase() returns "AUTO"
                if not data or data.strip() == "":
                    self.drive_safe_auton()
                elif "L" in data:
                    self.drive_to_left_goal()
                elif "R" in data:
                    self.drive_to_right_goal()
                else:
                    self.drive_safe_auton()
            case "ENDGAME":
                self.operator_controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 1.0)
            case "TELEOP":
                pass
   
    def drive_safe_auton(self):
        """Safe autonomous when no game data"""
        self.drive_forward(24)  # self.drive_forward(24) needs to make the robot drive forward a distance of 24 inches 
