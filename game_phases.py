import wpilib
from wpilib import DriverStation

def get_phase():
    time = DriverStation.getMatchTime()
    game_data = DriverStation.getGameSpecificMessage()  # In late versions of WPILib, the method DriverStation.getGameSpecificMessage() always returns a string object, never None in Python, even if there's no game data.

    # The code changes potentially None or falsy values into an empty string "", so any code that calls get_phase() isn't required to check for None before using the game_data.
    if not game_data:
        game_data = ""
    
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


class GamePhasesClass(wpilib.TimedRobot):  # This adds everything correctly so self works
    def robotInit(self):
        # Track when auto began so we can time out waiting for game data
        self.auto_start_time = 0.0

    def autonomousInit(self):
        self.auto_start_time = wpilib.Timer.getFPGATimestamp()

    def _has_valid_game_data(self, data: str) -> bool:
        # Example: for particular games that send "L" or "R"
        data = data.strip()
        return data in ("L", "R")
    
    def autonomousPeriodic(self):
        phase, data = get_phase()

        if phase != "AUTO":
            # Auto logic just belongs in auto
            return
        
        elapsed = wpilib.Timer.getFPGATimestamp() - self.auto_start_time

        if self._has_valid_game_data(data):
            # The data that is unique to the 2026 REBUILT GAME from the Field Management System (FMS) or Driver Station (DS) has passed validation, so your robot should now use that data to make decisions, such as which side to go to.
            if "L" in data:
                   self.drive_to_left_goal()
                elif "R" in data:
                    self.drive_to_right_goal()
                else:
                    # Shouldn't be hit because of _has_valid_game_data, but keep a safety net to catch errors or edge cases that shouldn't usually. occur.
                    self.drive_safe_auton()
            else:
                # No valid data yet: before timeout, you may choose to do nothing or a very secure action
            if not data.strip() or data not in ("L", "R"):
                if elapsed > 1.0:
                                # If the Field Management System (FMS) fails to send game data, or if the Driver Station (DS) receives corrupted or wrong data, your robot needs a backup plan instead of not doing anything.
                                self.drive_safe_auton()
                else:
                    # still in the waiting window; either do nothing
                    # or something extremely conservative
                    self.drive_safe_auton()  # or 'pass' if you actually want to wait
            else:
               
self.drive_to_left_goal()
                elif "R" in data:

self.drive_to_right_goal()
                else:

self.drive_safe_auton()
                    
    def teleopPeriodic(self):
        phase, data = get_phase()
        match phase:
            case "AUTO":  
                # Typically you won't run auto logic from teleopPeriodic,
                # but if you really want  this here, keep the equal fallback idea:
                if not data or data.strip() == "":
                    
self.drive_safe_auton()
                elif "L" in data:
                    
self.drive_to_left_goal()
                elif "R" in data:
                    
self.drive_to_right_goal()
                else:
                    
self.drive_safe_auton()
            case "ENDGAME":
                
self.operator_controller.setRumble(
    
wpilib.XboxController.RumbleType.kBothRum
ble, 1.0
                )
            case "TELEOP":
                pass
   
    def drive_safe_auton(self):
        """Safe autonomous when no game data"""
        self.drive_forward(24)  # self.drive_forward(24) needs to make the robot drive forward a distance of 24 inches 
