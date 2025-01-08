from commands2 import Command

class AutonomousCommand(Command):
    
    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        
    def initialize(self):
        pass

    def execute(self):
        # Autonomous routine example
        self.drivetrain.drive(0.6, 0.0)  # Move forward at 60% speed

    def end(self, interrupted):
        # Stop the robot when command ends
        self.drivetrain.drive(0.0, 0.0)

    def isFinished(self):
        # Run for a fixed duration (e.g., 3 seconds)
        return self.timeSinceInitialized() > 3.0

    def interrupted(self):
        self.end()


