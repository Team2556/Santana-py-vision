
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
        ...

        #initialize the Xbox conroller
        self.Drivercontroller = wpilib.XboxController(OIConstant.kDriver1ControllerPort)

    def update_sim(self, now, tm_diff):
        # Currently, the Python API for CTRE doesn't automatically detect the the
        # Sim driverstation status and enable the signals. So, for now, manually
        # feed the enable signal for double the set robot period.
        feed_enable(0.020 * 2)
        ...
        