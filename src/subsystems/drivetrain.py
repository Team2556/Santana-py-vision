import wpilib
import wpilib.drive
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
import commands2
from commands2 import Subsystem

import phoenix5

from constants import DriveConstant

...

class Drivetrain(Subsystem):
    def __init__(self):
        super().__init__()
        # Initialize drivetrain motors here

        self.frontLeftMotor = phoenix5.WPI_TalonSRX(DriveConstant.kLeftMotor1Port)
        SmartDashboard.putData("frontLeftMotor -from drivetrain", self.frontLeftMotor)
        self.frontLeftMotor.setInverted(self.right_invert_YN)
        ...
        self.robotDrive = wpilib.drive.DifferentialDrive(self.frontLeftMotor, self.right_motor)
        self.robotDrive.setMaxOutput(0.60)
    
    def periodic(self) -> None:
        """This method will be called once per scheduler run"""
        # Add code here that needs to run periodically
        
    def drive(self, speed, rotation):
        self.robotDrive.arcadeDrive(speed, rotation)
