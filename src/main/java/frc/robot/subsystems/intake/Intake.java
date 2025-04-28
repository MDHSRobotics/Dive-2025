package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    public enum IntakeArmPositions {
        STOWED,
        ON_CORAL_PICKUP,
        GROUND_PICKUP,
        PROCESSOR
    }

    private final SparkFlex m_armMotor = new SparkFlex(ARM_ID, MotorType.kBrushless);
    private final SparkMax m_flywheelLeftMotor = new SparkMax(WHEEL_LEFT_ID, MotorType.kBrushless);
    private final SparkMax m_flywheelRightMotor = new SparkMax(WHEEL_RIGHT_ID, MotorType.kBrushless);

    /*Break Beam Sensor */
    private final RelativeEncoder m_flywheelEncoder = m_flywheelLeftMotor.getEncoder();

    private final SparkClosedLoopController m_armController = m_armMotor.getClosedLoopController();

    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_table = m_inst.getTable("Intake");
    private final DoubleEntry m_flywheelSpeedEntry =
            m_table.getDoubleTopic("Flywheel Speed").getEntry(1);
    private final DoublePublisher m_targetPositionPub =
            m_table.getDoubleTopic("Target Position (radians)").publish();
    // private final DoubleEntry pGainEntry =
    //         table.getDoubleTopic("Arm P Gain").getEntry(K_P, PubSubOption.excludeSelf(true));

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
        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).p(K_P);
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
        flywheelConfig
                .signals
                .appliedOutputPeriodMs(5)
                .primaryEncoderVelocityPeriodMs(10)
                .primaryEncoderVelocityAlwaysOn(true);
        m_flywheelLeftMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelConfig.follow(m_flywheelLeftMotor, true);
        m_flywheelRightMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // You need to publish a value for the entry to appear in NetworkTables
        m_flywheelSpeedEntry.set(1);

        // pGainEntry.set(K_P);
        // inst.addListener(pGainEntry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
        //     SparkFlexConfig tempConfig = new SparkFlexConfig();
        //     tempConfig.closedLoop.p(event.valueData.value.getDouble());
        //     m_armMotor.configureAsync(tempConfig, ResetMode.kNoResetSafeParameters,
        // PersistMode.kNoPersistParameters);
        // });
    }

    private boolean wheelsAreStopped() {
        return MathUtil.isNear(0, m_flywheelEncoder.getVelocity(), 0.001);
    }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_armMotor.stopMotor();
                    m_flywheelLeftMotor.stopMotor();
                })
                .andThen(Commands.idle(this));
    }

    public Command armTestCommand(DoubleSupplier armPowerSupplier) {
        return this.run(() -> m_armMotor.set(armPowerSupplier.getAsDouble() * 0.25))
                .finallyDo(m_armMotor::stopMotor);
    }

    public Command wheelsBackwardsCommand() {
        return this.runOnce(() -> m_flywheelLeftMotor.set(m_flywheelSpeedEntry.get() * -0.25))
                .andThen(Commands.idle(this))
                .finallyDo(m_flywheelLeftMotor::stopMotor);
    }

    public Command runWheelsCommand() {
        return this.runOnce(() -> m_flywheelLeftMotor.set(0.5))
                .andThen(Commands.idle(this))
                .finallyDo(m_flywheelLeftMotor::stopMotor);
    }

    public Command runWheelsSlowCommand() {
        return this.runOnce(() -> m_flywheelLeftMotor.set(0.25)).andThen(Commands.idle(this));
    }

    public Command setArmPositionCommand(IntakeArmPositions armPosition) {
        return this.runOnce(() -> {
                    double position;
                    if (armPosition == IntakeArmPositions.STOWED) {
                        position = ARM_MAX_LIMIT;
                    } else if (armPosition == IntakeArmPositions.ON_CORAL_PICKUP) {
                        position = ON_CORAL_PICKUP_POSITION;
                    } else if (armPosition == IntakeArmPositions.GROUND_PICKUP) {
                        position = GROUND_PICKUP_POSITION;
                    } else {
                        position = PROCESSOR_POSITION;
                    }
                    m_armController.setReference(position, ControlType.kPosition);
                    m_targetPositionPub.set(position);
                })
                .andThen(Commands.idle(this));
    }
}
