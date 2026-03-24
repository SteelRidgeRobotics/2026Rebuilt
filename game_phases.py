import wpilib
from wpilib import DriverStation

def get_phase():
    time = DriverStation.getMatchTime()
    game_data = DriverStation.getGameSpecificMessage()  # always a string in recent WPILib [web:4]

    # Normalize game data so caller doesn't worry about None
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
        # Track when auto started so we can time out waiting for game data
        self.auto_start_time = 0.0

    def autonomousInit(self):
        self.auto_start_time = wpilib.Timer.getFPGATimestamp()

    def _has_valid_game_data(self, data: str) -> bool:
        # Example: for games that send "L" or "R" only
        data = data.strip()
        return data in ("L", "R")
    
    def autonomousPeriodic(self):
        phase, data = get_phase()

        if phase != "AUTO":
            # Auto logic only belongs in auto
            return
        
        elapsed = wpilib.Timer.getFPGATimestamp() - self.auto_start_time

        if self._has_valid_game_data(data):
            # We have a valid FMS / DS message, use it
            if "L" in data:
                   self.drive_to_left_goal()
                elif "R" in data:
                    self.drive_to_right_goal()
                else:
                    # Should not be hit because of _has_valid_game_data, but keep a safety net
                    self.drive_safe_auton()
            else:
                # No valid data yet: before timeout, you might choose to do nothing or a very safe action
            if not data.strip() or data not in ("L", "R"):
                if elapsed > 1.0:
                    # after 1s with no valid data, run fallback
                    self.drive_safe_auton()
                else:
                    # still within the waiting window; either do nothing
                    # or something extremely conservative
                    self.drive_safe_auton()  # or 'pass' if you really want to wait
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
                # Normally you won't run auto logic from teleopPeriodic,
                # but if you really want  this here, keep the same fallback idea:
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
