from typing import List
from typing import Callable
from wpimath.kinematics._kinematics import ChassisSpeeds
from subsystems.drive.swerve_module import SwerveModule
from commands2 import Subsystem
from commands2 import Command
from phoenix6.hardware import Pigeon2
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
from wolverine_sim.rev.spark_max_simulation import spark_max_sim
from wolverine_sim.rev.relative_encoder_simulation import relative_encoder_sim
from wolverine_sim.phoenix6.pigeon2_simulation import pigeon2_sim
from wolverine_sim.wpilib.analog_encoder_simulation import analog_encoder_sim
import subsystems.drive.drive_constants as drive_constants
import wpimath.kinematics
import wpilib

class DriveSubsystem(Subsystem):

    def __init__(
        self,
        modules: List[SwerveModule],
        gyro_id: int,
    ):
        """
        :param modules: List containing the 4 SwerveModule objects for the drive base
        :param gyro_id: The CAN ID of the gyroscope
        """
        # Creating the list of modules from the modules parameter
        self.modules = modules

        # Creating a gyro object from the gyro id
        self.gyro = Pigeon2(gyro_id)

        # Creating a kinematics object based on the module translation constants
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(drive_constants.X_MODULE_TRANSLATION, drive_constants.Y_MODULE_TRANSLATION),
            Translation2d(drive_constants.X_MODULE_TRANSLATION, -drive_constants.Y_MODULE_TRANSLATION),
            Translation2d(-drive_constants.X_MODULE_TRANSLATION, drive_constants.Y_MODULE_TRANSLATION),
            Translation2d(-drive_constants.X_MODULE_TRANSLATION, -drive_constants.Y_MODULE_TRANSLATION),
        )

        if wpilib.RobotBase.isSimulation():

            # Iterating through all the modules and registering each device
            for x in range(0, len(modules)):
                spark_max_sim.register_device(modules[x].drive_motor.getDeviceId(), drive_constants.DRIVE_MUJOCO_IDS[x])
                relative_encoder_sim.register_device(id(modules[x].drive_encoder), drive_constants.DRIVE_MUJOCO_IDS[x])

                spark_max_sim.register_device(modules[x].steer_motor.getDeviceId(), drive_constants.STEER_MUJOCO_IDS[x])
                analog_encoder_sim.register_device(modules[x].absolute_encoder.getChannel(), drive_constants.STEER_MUJOCO_IDS[x])

            # Registering the gyro to simulation
            pigeon2_sim.register_device(id(self.gyro), drive_constants.GYRO_MUJOCO_ID)


    def get_drive_command(
            self,
            vertical_supplier: Callable[[], float],
            horizontal_supplier: Callable[[], float],
            rotation_supplier: Callable[[], float]
    ) -> Command:
        """
        Method to get the drive command for the drive base
        :param vertical_supplier: The method to get the vertical speed
        :param horizontal_supplier: The method to get the horizontal speed
        :param rotation_supplier: The method to get the rotational speed
        :return:
        """
        # Creating and returning the command object using drive method
        return self.runOnce(
            lambda: self.drive(ChassisSpeeds(vertical_supplier(), horizontal_supplier(), rotation_supplier()))
        )

    def drive(self, target_speed: wpimath.kinematics.ChassisSpeeds):
        """
        Method to control the drive base
        :param target_speed: The target linear and angular velocity packaged as a ChasisSpeeds
        :return:
        """
        target_states = self.kinematics.toSwerveModuleStates(target_speed)
        for x in range(0, len(self.modules)):
            self.modules[x].set_state(target_states[x])

