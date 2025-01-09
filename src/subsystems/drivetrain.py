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
        self.right_invert_YN = True

        self.frontLeftMotor = phoenix5.WPI_TalonSRX(DriveConstant.kLeftMotor1Port)
        SmartDashboard.putData("frontLeftMotor -from drivetrain", self.frontLeftMotor)
        self.rearLeftMotor = phoenix5.WPI_TalonSRX(DriveConstant.kLeftMotor2Port)
        SmartDashboard.putData("rearLeftMotor -from drivetrain", self.rearLeftMotor)
        self.frontRightMotor = phoenix5.WPI_TalonSRX(DriveConstant.kRightMotor1Port)
        SmartDashboard.putData("frontRightMotor -from drivetrain", self.frontRightMotor)
        self.rearRightMotor = phoenix5.WPI_TalonSRX(DriveConstant.kRightMotor2Port)
        SmartDashboard.putData("rearRightMotor -from drivetrain", self.rearRightMotor)


        self.frontRightMotor.setInverted(self.right_invert_YN)
        self.rearRightMotor.setInverted(self.right_invert_YN)
        ...
        self.robotDrive = wpilib.drive.MecanumDrive(self.frontLeftMotor, self.rearLeftMotor, self.frontRightMotor, self.rearRightMotor)
        self.robotDrive.setMaxOutput(0.60)
    
    def periodic(self) -> None:
        """This method will be called once per scheduler run"""
        # Add code here that needs to run periodically
        
    def drive(self, xSpeed: float, ySpeed: float, zRotation: float) -> None:
        self.robotDrive.driveCartesian(xSpeed, ySpeed, zRotation)

    
