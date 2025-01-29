package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private final SparkFlex m_armMotor = new SparkFlex(IntakeConstants.ARM_ID, MotorType.kBrushless);

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Intake() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT)
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        config.encoder
                .positionConversionFactor(IntakeConstants.ARM_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(IntakeConstants.ARM_VELOCITY_CONVERSION_FACTOR);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(IntakeConstants.K_P)
                .d(IntakeConstants.K_D);
        config.closedLoop
                .maxMotion
                .maxVelocity(IntakeConstants.MAX_VELOCITY)
                .maxAcceleration(IntakeConstants.MAX_ACCELERATION);
        config.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true);
        m_armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_armMotor.set(0);
                })
                .andThen(Commands.idle(this));
    }

    public Command motorTestCommand(DoubleSupplier leftMotorPowerSupplier) {
        return this.run(() -> {
            m_armMotor.set(leftMotorPowerSupplier.getAsDouble());
        });
    }
}
