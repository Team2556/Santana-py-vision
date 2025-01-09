
import wpilib
from wpilib import SmartDashboard
import wpilib.simulation
from wpilib.simulation import (PWMSim, AnalogGyroSim,)

from wpimath.kinematics import (MecanumDriveKinematics,
                                # MecanumDriveKinematicsBase,
                                # MecanumDriveOdometry,
                                # MecanumDriveOdometryBase,
                                # MecanumDriveWheelPositions,
                                MecanumDriveWheelSpeeds,
                                DifferentialDriveKinematics,
                                DifferentialDriveOdometry,  
                                DifferentialDriveWheelSpeeds,
                                )
from wpimath.geometry import (Pose2d, Rotation2d, Translation2d)
# from wpimath.estimator import MecanumDrivePoseEstimator 
# from robotpy_ext.common_drivers import navx
from constants import (DriveConstant, OIConstant)
from phoenix6.unmanaged import feed_enable

from pyfrc.physics.drivetrains import FourMotorDrivetrain
from pyfrc.physics.units import units

class PhysicsEngine:
    def __init__(self, physics_controller, robot: "MyRobot"): # type: ignore
        self.physics_controller = physics_controller
        self.robot = robot
        self.robotDrive = robot.robotDrive
        

        # Initialize motor controllers
        self.right_invert_YN = robot.robotDrive.right_invert_YN

        # useing phoenix5 sim collection off of our actual motors
        self.frontLeftMotor = robot.robotDrive.frontLeftMotor.getSimCollection()
        self.rearLeftMotor = robot.robotDrive.rearLeftMotor.getSimCollection()
        self.frontRightMotor = robot.robotDrive.frontRightMotor.getSimCollection()
        self.rearRightMotor = robot.robotDrive.rearRightMotor.getSimCollection()
        
        # self.frontRightMotor.setInverted(self.right_invert_YN)
        # self.rearRightMotor.setInverted(self.right_invert_YN)

        #initialize the Xbox conroller
        self.Drivercontroller = wpilib.XboxController(OIConstant.kDriver1ControllerPort)

    def update_sim(self, now, tm_diff):
        # Currently, the Python API for CTRE doesn't automatically detect the the
        # Sim driverstation status and enable the signals. So, for now, manually
        # feed the enable signal for double the set robot period.
        feed_enable(0.020 * 2)
        frontLeftMotor_speed = self.frontLeftMotor.getMotorOutputLeadVoltage() /12 #assuming 12 volts for now (could sim the battery :)
        rearLeftMotor_speed = self.rearLeftMotor.getMotorOutputLeadVoltage() /12
        frontRightMotor_speed = self.frontRightMotor.getMotorOutputLeadVoltage() /12
        rearRightMotor_speed = self.rearRightMotor.getMotorOutputLeadVoltage() /12

        wheel_speeds = MecanumDriveWheelSpeeds(
                    frontLeftMotor_speed,
                    frontRightMotor_speed,
                    rearLeftMotor_speed,
                    rearRightMotor_speed
                )
        # Create an odometry object
        drivetrain_kinematics = MecanumDriveKinematics(
                        Translation2d(DriveConstant.kWheelBase / 2, DriveConstant.kTrackWidth / 2),
                        Translation2d(DriveConstant.kWheelBase / 2, -DriveConstant.kTrackWidth / 2),
                        Translation2d(-DriveConstant.kWheelBase / 2, DriveConstant.kTrackWidth / 2),
                        Translation2d(-DriveConstant.kWheelBase / 2, -DriveConstant.kTrackWidth / 2)
                    )
        chassis_speeds = drivetrain_kinematics.toChassisSpeeds(wheel_speeds)
        # Update the physics controller with the new state
        self.physics_controller.drive(chassis_speeds, tm_diff)


        ...
        