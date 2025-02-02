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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CatcherConstants;
import java.util.function.DoubleSupplier;

public class Catcher extends SubsystemBase {
    private final SparkFlex m_armMotor = new SparkFlex(CatcherConstants.ARM_ID, MotorType.kBrushless);
    private final SparkFlex m_wheelsMotor = new SparkFlex(CatcherConstants.WHEELS_ID, MotorType.kBrushless);
    // private final DigitalInput m_armBeamBreak = new DigitalInput(9);

    private final SysIdRoutine m_armRoutine =
            new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_armMotor::setVoltage, null, this));

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Catcher() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(CatcherConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
        config.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true);
        m_wheelsMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.encoder
                .positionConversionFactor(CatcherConstants.ARM_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(CatcherConstants.ARM_VELOCITY_CONVERSION_FACTOR);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(CatcherConstants.K_P)
                .d(CatcherConstants.K_D);
        m_armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_armMotor.set(0);
                    m_wheelsMotor.set(0);
                })
                .andThen(Commands.idle(this));
    }

    public Command motorTestCommand(DoubleSupplier leftMotorPowerSupplier, DoubleSupplier rightMotorPowerSupplier) {
        return this.run(() -> {
            m_armMotor.set(leftMotorPowerSupplier.getAsDouble());
            m_wheelsMotor.set(rightMotorPowerSupplier.getAsDouble());
        });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_armRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_armRoutine.dynamic(direction);
    }
}
