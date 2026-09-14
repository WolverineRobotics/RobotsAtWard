import rev
import wpilib
import wpimath.controller
import subsystems.drive.drive_constants as drive_constants
import wpimath
import math


class SwerveModule():


    def __init__(
            self,
            drive_motor_id: int,
            steer_motor_id: int,
            encoder_id: int,
            encoder_offset: float,
    ) -> None:
        """
        :param drive_motor_id: The CAN ID of the drive motor
        :param steer_motor_id: The CAN ID of the steer motor
        :param encoder_id: The analog port on the rio for the absolute encoder
        :param encoder_offset: The absolute encoder offset for the module
        """

        # Creating objects form the drive and steer motor in the module
        self.drive_motor = rev.SparkMax(drive_motor_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.steer_motor = rev.SparkMax(steer_motor_id, rev.SparkLowLevel.MotorType.kBrushless)

        # Creating the absolute encoder with the correct port, max input, and offset
        self.analog_input = wpilib.AnalogInput(encoder_id)
        self.absolute_encoder = wpilib.AnalogEncoder(self.analog_input, 360, encoder_offset)

        # Getting the relative encoder object form the drive motor
        self.drive_encoder = self.drive_motor.getEncoder()

        # Creating the drive and steer PID controllers using the constants from the drive_constants file
        self.drive_pid = wpimath.controller.PIDController(
            drive_constants.DRIVE_KP,
            drive_constants.DRIVE_KI,
            drive_constants.DRIVE_KD
        )

        self.steer_pid = wpimath.controller.PIDController(
            drive_constants.STEER_KP,
            drive_constants.STEER_KI,
            drive_constants.STEER_KD
        )


    def set_state(self, target_state: wpimath.kinematics.SwerveModuleState) -> None:
        """
        Method to set the swerve module to a given angle and speed.
        :param target_state: The target angle and speed packaged as a SwerveModuleStateObject
        :return:
        """
        # Getting the target speed and angle from the target state object
        target_speed: float = target_state.speed
        target_angle: float = target_state.angle.degrees() + 180 # Absolute encoder goes from 0 -> 360

        # Setting the drive and steer motors with value calculated from the PID controllers
        self.drive_motor.set(
            self.drive_pid.calculate(self._get_module_speed(), target_speed)
        )

        self.steer_motor.set(
            self.steer_pid.calculate(self._get_module_angle(), target_angle)
        )

    def _get_module_angle(self) -> wpimath.geometry.Rotation2d:
        """
        Method to get the current angle of the swerve module
        :return: The current module angle as a Rotation2d
        """
        return self.absolute_encoder.get()

    def _get_module_speed(self) -> float:
        """
        Method to get the current speed of the swerve module
        :return: The current speed of the module in meters per second
        """
        return (
            # Converting the encoder reading from RPM -> Meters per Sec using wheel radius
            self.drive_encoder.getVelocity() * (2 * math.pi / 60) * drive_constants.wheel_radius
        )