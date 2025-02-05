package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private final SparkFlex m_armMotor = new SparkFlex(ARM_ID, MotorType.kBrushless);
    private final SparkMax m_wheelLeftMotor = new SparkMax(WHEEL_LEFT_ID, MotorType.kBrushless);
    private final SparkMax m_wheelRightMotor = new SparkMax(WHEEL_RIGHT_ID, MotorType.kBrushless);

    private final SysIdRoutine m_armRoutine =
            new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_armMotor::setVoltage, null, this));

    private final ArmFeedforward m_armFeedforward = new ArmFeedforward(K_S, K_G, K_V, K_A);

    private final TrapezoidProfile m_armProfile = new TrapezoidProfile(ARM_ANGULAR_MOTION_CONSTRAINTS);

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Intake() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(ARM_CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(true);
        config.encoder
                .positionConversionFactor(ARM_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(ARM_VELOCITY_CONVERSION_FACTOR);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(K_P).d(K_D);
        config.signals
                .primaryEncoderPositionPeriodMs(10)
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(10)
                .primaryEncoderVelocityAlwaysOn(true);
        m_armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig wheelConfig = new SparkMaxConfig();
        wheelConfig.smartCurrentLimit(WHEEL_CURRENT_LIMIT).idleMode(IdleMode.kBrake);
        wheelConfig
                .encoder
                .positionConversionFactor(WHEEL_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(WHEEL_VELOCITY_CONVERSION_FACTOR);
        wheelConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true);
        m_wheelLeftMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_wheelRightMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_armMotor.stopMotor();
                    ;
                })
                .andThen(Commands.idle(this));
    }

    public Command armTestCommand(DoubleSupplier armPowerSupplier) {
        return this.run(() -> {
            m_armMotor.set(armPowerSupplier.getAsDouble());
        });
    }

    public Command wheelsTestCommand() {
        return this.run(() -> {
            m_wheelLeftMotor.set(0.5);
            m_wheelRightMotor.set(0.5);
        });
    }

    public Command wheelsBackwardsTestCommand() {
        return this.run(() -> {
            m_wheelLeftMotor.set(-0.5);
            m_wheelRightMotor.set(-0.5);
        });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_armRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_armRoutine.dynamic(direction);
    }
}
