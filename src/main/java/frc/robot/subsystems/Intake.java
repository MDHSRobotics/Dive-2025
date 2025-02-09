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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private final SparkFlex m_armMotor = new SparkFlex(ARM_ID, MotorType.kBrushless);
    private final SparkMax m_flywheelLeftMotor = new SparkMax(WHEEL_LEFT_ID, MotorType.kBrushless);
    private final SparkMax m_flywheelRightMotor = new SparkMax(WHEEL_RIGHT_ID, MotorType.kBrushless);

    private final SysIdRoutine m_armRoutine =
            new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_armMotor::setVoltage, null, this));

    private final SimpleMotorFeedforward m_armFeedforward = new SimpleMotorFeedforward(K_S, K_V, K_A);

    private final TrapezoidProfile m_armProfile = new TrapezoidProfile(ARM_ANGULAR_MOTION_CONSTRAINTS);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Intake");
    private final DoubleEntry flyheelSpeedEntry =
            table.getDoubleTopic("Flywheel Speed").getEntry(1);

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Intake() {
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.smartCurrentLimit(ARM_CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(true);
        armConfig
                .encoder
                .positionConversionFactor(ARM_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(ARM_VELOCITY_CONVERSION_FACTOR);
        armConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(K_P)
                .d(K_D);
        armConfig
                .signals
                .primaryEncoderPositionPeriodMs(10)
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(10)
                .primaryEncoderVelocityAlwaysOn(true);
        m_armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig
                .smartCurrentLimit(WHEEL_CURRENT_LIMIT)
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        flywheelConfig
                .encoder
                .positionConversionFactor(WHEEL_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(WHEEL_VELOCITY_CONVERSION_FACTOR);
        flywheelConfig.signals.appliedOutputPeriodMs(5);
        m_flywheelLeftMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelConfig.follow(m_flywheelLeftMotor, true);
        m_flywheelRightMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // You need to publish a value for the entry to appear in NetworkTables
        flyheelSpeedEntry.set(1);
    }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_armMotor.stopMotor();
                    m_flywheelLeftMotor.stopMotor();
                })
                .andThen(Commands.idle(this));
    }

    public Command armTestCommand(DoubleSupplier armPowerSupplier) {
        return this.run(() -> m_armMotor.set(armPowerSupplier.getAsDouble() * 0.25));
    }

    public Command wheelsTestCommand() {
        return this.run(() -> m_flywheelLeftMotor.set(flyheelSpeedEntry.get() * 0.25));
    }

    public Command wheelsBackwardsTestCommand() {
        return this.run(() -> m_flywheelLeftMotor.set(-flyheelSpeedEntry.get() * 0.25));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_armRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_armRoutine.dynamic(direction);
    }
}
