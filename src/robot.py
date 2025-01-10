import wpilib
from wpilib import (SmartDashboard, Field2d)
from wpimath.geometry import Rotation2d
import wpilib.cameraserver
import commands2
from commands2 import CommandScheduler
from constants import (DriveConstant,
                       OIConstant,
                       )
from subsystems.drivetrain import Drivetrain
from commands.autonomous import AutonomousCommand
from commands.teleop import TeleopCommand
#start camera server


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        """Robot-wide initialization code should go here."""
        CommandScheduler.getInstance().run()
        
        #kickoff vision
        wpilib.CameraServer.launch("vision.py:main")

        # Instantiate subsystems
        self.robotDrive = Drivetrain()
        # Instantiate OI
        self.driverController = commands2.button.CommandXboxController(
            OIConstant.kDriver1ControllerPort)
        # Configure the button bindings
        self.ConfigureButtonBindings()
        
        # Instantiate default commands
        # Sanatana drive setup with the wide intake as front
        self.robotDrive.setDefaultCommand(commands2.cmd.run(lambda: self.robotDrive.drive(xSpeed=-self.driverController.getLeftY() ,
                                                                                          ySpeed=self.driverController.getLeftX(),
                                                                                          zRotation=self.driverController.getRightX())#,  Rotation2d(0))
                                                            , self.robotDrive))
    
        #region SmartDashboard init
        SmartDashboard.putData(CommandScheduler.getInstance())
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field) #end up viewing in Glass
        #endregion SmartDashBoard init


    def autonomousInit(self):
        """Runs once when autonomous mode is enabled."""
        

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopInit(self):
        """Runs once when teleop mode is enabled."""


    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""

    def disabledInit(self):
        """Runs once when the robot is disabled."""
        CommandScheduler.getInstance().cancelAll()

    def ConfigureButtonBindings(self):
        # self.driverController.povLeft().onTrue(lambda: self.robotDrive.slowLeft(self.driverController))
        # move to commands... OnlyFrontLeft = commands2.SequentialCommandGroup(
        #     commands2.cmd.run(lambda: self.robotDrive.OnlyFrontLeft()).raceWith(
        #         commands2.WaitCommand(2.2)))
        # self.driverController.x().onTrue(OnlyFrontLeft)
        ...
