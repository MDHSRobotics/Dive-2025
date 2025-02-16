package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    public enum IntakeArmPositions {
        UP,
        ON_CORAL_PICKUP,
        GROUND_PICKUP
    }

    private final SparkFlex m_armMotor = new SparkFlex(ARM_ID, MotorType.kBrushless);
    private final SparkMax m_flywheelLeftMotor = new SparkMax(WHEEL_LEFT_ID, MotorType.kBrushless);
    private final SparkMax m_flywheelRightMotor = new SparkMax(WHEEL_RIGHT_ID, MotorType.kBrushless);

    private final DigitalInput m_armBeamBreak = new DigitalInput(ARM_BEAM_BEAK_DIO_CHANNEL);

    private final SparkClosedLoopController m_armController = m_armMotor.getClosedLoopController();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Intake");
    private final DoubleEntry flyheelSpeedEntry =
            table.getDoubleTopic("Flywheel Speed").getEntry(1);
    private final BooleanPublisher beamBreakPub =
            table.getBooleanTopic("Beam Broken").publish(PubSubOption.sendAll(true));

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Intake() {
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.smartCurrentLimit(ARM_CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(true);
        armConfig
                .softLimit
                .forwardSoftLimit(ARM_MAX_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ARM_MIN_LIMIT)
                .reverseSoftLimitEnabled(true);
        armConfig
                .absoluteEncoder
                .positionConversionFactor(ARM_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(ARM_VELOCITY_CONVERSION_FACTOR)
                .zeroOffset(ARM_ZERO_OFFSET);
        armConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(K_P)
                .d(K_D);
        armConfig
                .signals
                .absoluteEncoderPositionPeriodMs(10)
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(10)
                .absoluteEncoderVelocityAlwaysOn(true);
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

    @Override
    public void periodic() {
        beamBreakPub.set(!m_armBeamBreak.get());
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

    /**
     * Runs the wheels until the beam is broken.
     */
    public Command wheelsTestCommand() {
        return this.run(() -> m_flywheelLeftMotor.set(flyheelSpeedEntry.get() * 0.25));
    }

    public Command wheelsBackwardsTestCommand() {
        return this.run(() -> m_flywheelLeftMotor.set(-flyheelSpeedEntry.get() * 0.25));
    }
}
