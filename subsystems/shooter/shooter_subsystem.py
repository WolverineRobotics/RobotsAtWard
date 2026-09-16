import wpimath.controller as controller
from commands2 import Subsystem
from commands2 import Command
import rev
from subsystems.shooter import shooter_constansts


class ShooterSubsystem(Subsystem):

    def __init__(
            self,
            flywheel_id: int,
            indexer_id: int,
            flywheel_inv: bool,
            indexer_inv: bool
    ):
        self.flywheel_motor = rev.SparkFlex(flywheel_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.indexer_motor = rev.SparkMax(indexer_id, rev.SparkLowLevel.MotorType.kBrushless)

        flywheel_config = rev.SparkFlexConfig()
        flywheel_config.inverted(flywheel_inv)
        flywheel_config.smartCurrentLimit(shooter_constansts.FLYWHEEL_CURRENT_LIMIT)

        indexer_config = rev.SparkMaxConfig()
        indexer_config.inverted(indexer_inv)
        indexer_config.smartCurrentLimit(shooter_constansts.INDEXER_CURRENT_LIMIT)

        self.flywheel_motor.configure(
            flywheel_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters
        )

        self.indexer_motor.configure(
            indexer_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters
        )

        self.flywheel_feedforward = controller.SimpleMotorFeedforwardRadians(
            shooter_constansts.FLYWHEEL_KS,
            shooter_constansts.FLYWHEEL_KV
        )

    def set_flywheel_speed(self, speed: float):
        self.flywheel_motor.setVoltage(
            self.flywheel_feedforward.calculate(
                speed
            )
        )

    def get_flywheel_speed(self) -> float:
        return self.flywheel_motor.getEncoder().getVelocity()

    def spin_indexer(self):
        self.indexer_motor.set(shooter_constansts.INDEXER_SPEED)

    def shoot(self, speed: float):
        self.set_flywheel_speed(speed)
        if self.get_flywheel_speed() >= abs(speed):
            self.spin_indexer()

    def get_shoot_command(self, speed) -> Command:
        return self.runOnce(
            lambda: self.shoot(speed)
        )


