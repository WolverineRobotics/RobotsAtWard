import commands2
from commands2 import Subsystem
from commands2 import Command
import intake_constants
import rev

class IntakeSubsystem(Subsystem):

    def __init__(
            self,
            right_pivot_id: int,
            left_pivot_id: int,
            roller_id: int,
            right_pivot_inv: bool,
            left_pivot_inv: bool
    ):
        self.right_pivot_motor = rev.SparkMax(right_pivot_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.left_pivot_motor = rev.SparkMax(left_pivot_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.roller_motor = rev.SparkMax(roller_id, rev.SparkLowLevel.MotorType.kBrushless)

        right_pivot_config = rev.SparkMaxConfig()
        left_pivot_config = rev.SparkMaxConfig()
        roller_config = rev.SparkMaxConfig()

        right_pivot_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        right_pivot_config.inverted(right_pivot_inv)
        right_pivot_config.smartCurrentLimit(intake_constants.PIVOT_CURRENT_LIMIT, intake_constants.PIVOT_CURRENT_LIMIT)

        left_pivot_config.follow(right_pivot_id, True)
        left_pivot_config.inverted(left_pivot_inv)
        left_pivot_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        left_pivot_config.smartCurrentLimit(intake_constants.PIVOT_CURRENT_LIMIT, intake_constants.PIVOT_CURRENT_LIMIT)

        roller_config.smartCurrentLimit(intake_constants.ROLLER_CURRENT_LIMIT, intake_constants.ROLLER_CURRENT_LIMIT)

        self.right_pivot_motor.configure(right_pivot_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.left_pivot_motor.configure(left_pivot_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.roller_id.configure(roller_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)



    def at_bumpers(self) -> bool:
        return self.right_pivot_motor.getOutputCurrent() >= intake_constants.STRESSED_INTAKE_CURRENT_DRAW

    def pivot_intake(self) -> None:
        self.right_pivot_motor.set(intake_constants.INTAKE_PIVOT_SPEED)


    def get_pivot_command(self) -> Command:
        return commands2.FunctionalCommand(
            lambda *args: None,
            lambda: self.pivot_intake(),
            lambda *args: None,
            lambda: self.at_bumpers()
        )

    def spin_rollers(self) -> None:
        self.roller_motor.set(intake_constants.ROLLER_SPEED)

    def get_intake_command(self) -> Command:
        return self.runOnce(
            lambda: self.spin_rollers()
        )