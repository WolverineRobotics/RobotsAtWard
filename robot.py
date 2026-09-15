import commands2
from commands2 import CommandScheduler
from subsystems.drive.drive_subsystem import DriveSubsystem
from subsystems.drive.swerve_module import SwerveModule
from subsystems.intake import intake_constants
from subsystems.intake.intake_subsystem import IntakeSubsystem
from subsystems.shooter.shooter_subsystem import ShooterSubsystem
from subsystems.shooter import shooter_constansts
from wolverine_sim.robot_simulation import rs
from wolverine_sim.rev.spark_max_simulation import spark_max_sim
from wolverine_sim.phoenix6.pigeon2_simulation import pigeon2_sim
from wolverine_sim.wpilib.analog_encoder_simulation import analog_encoder_sim
from wolverine_sim.rev.relative_encoder_simulation import relative_encoder_sim
import subsystems.drive.drive_constants as drive_constants
import wpilib

class Robot(wpilib.TimedRobot):

    def robotInit(self) -> None:
        # Creating the DriveSubsystem with constants from each swerve module
        self.drive_subsystem = DriveSubsystem(
            [
                SwerveModule(
                    drive_constants.FRONT_LEFT_DRIVE_CAN_ID,
                    drive_constants.FRONT_LEFT_STEER_CAN_ID,
                    drive_constants.FRONT_LEFT_ENCODER_ID,
                    drive_constants.FRONT_LEFT_ENCODER_OFFSET
                ),

                SwerveModule(
                    drive_constants.FRONT_RIGHT_DRIVE_CAN_ID,
                    drive_constants.FRONT_RIGHT_STEER_CAN_ID,
                    drive_constants.FRONT_RIGHT_ENCODER_ID,
                    drive_constants.FRONT_RIGHT_ENCODER_OFFSET
                ),

                SwerveModule(
                    drive_constants.BACK_LEFT_DRIVE_CAN_ID,
                    drive_constants.BACK_LEFT_STEER_CAN_ID,
                    drive_constants.BACK_LEFT_ENCODER_ID,
                    drive_constants.BACK_LEFT_ENCODER_OFFSET
                ),

                SwerveModule(
                    drive_constants.BACK_RIGHT_DRIVE_CAN_ID,
                    drive_constants.BACK_RIGHT_STEER_CAN_ID,
                    drive_constants.BACK_RIGHT_ENCODER_ID,
                    drive_constants.BACK_RIGHT_ENCODER_OFFSET
                ),
            ],

            drive_constants.GYRO_CAN_ID
        )

        self.intake_subsystem = IntakeSubsystem(
            intake_constants.RIGHT_PIVOT_ID,
            intake_constants.LEFT_PIVOT_ID,
            intake_constants.ROLLER_ID,
            intake_constants.RIGHT_PIVOT_INVERTED,
            intake_constants.LEFT_PIVOT_INVERTED
        )

        self.shooter_subsystem = ShooterSubsystem(
            shooter_constansts.FLYWHEEL_ID,
            shooter_constansts.INDEXER_ID,
            shooter_constansts.FLYWHEEL_INVERTED,
            shooter_constansts.INDEXER_INVERTED
        )



        # Creating the drive controller on port 0 which is our team's standard
        self.drive_controller = commands2.button.CommandXboxController(0)

        # Creating the operator controller on port 1 which is our team's standard
        self.op_controller = commands2.button.CommandXboxController(1)

        # Configuring the controls for the robot
        self.configure_bindings()


        if wpilib.RobotBase.isSimulation():
            # Initializing the mujoco simulation if the robot is being simulated
            rs.initialize("robot/scene.xml")

            # Setting up the wrappers for all the devices used on the robot
            spark_max_sim.setup_wrappers()
            pigeon2_sim.setup_wrappers()
            analog_encoder_sim.setup_wrappers()
            relative_encoder_sim.setup_wrappers()

    def configure_bindings(self) -> None:
        # Setting the default command of the drive subsystem to be the drive command
        # and passing the methods from the drive controller as parameters
        self.drive_subsystem.setDefaultCommand(
            self.drive_subsystem.get_drive_command(
                self.drive_controller.getLeftY,
                self.drive_controller.getLeftX,
                self.drive_controller.getRightX
            )
        )

        self.op_controller.a().whileTrue(
            self.intake_subsystem.get_intake_command()
        )

        self.op_controller.b().onTrue(
            self.intake_subsystem.get_pivot_command()
        )



    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def teleopPeriodic(self) -> None:
        pass

    def _simulationInit(self) -> None:
        # Starting the simulation
        rs.start()

    def _simulationPeriodic(self) -> None:
        # Moving the simulation forward by 1 step
        rs.step()
