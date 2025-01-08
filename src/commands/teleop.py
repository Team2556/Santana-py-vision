from commands2 import Command

class TeleopCommand(Command):
    
    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        
    def execute(self):
        # Placeholder for teleop control logic
        speed = 0.5  # Example fixed speed, replace with joystick input
        rotation = 0.0  # Example fixed rotation
        self.drivetrain.drive(speed, rotation)

    def isFinished(self):
        return False

    def end(self):
        pass

    def interrupted(self):
        self.end()


