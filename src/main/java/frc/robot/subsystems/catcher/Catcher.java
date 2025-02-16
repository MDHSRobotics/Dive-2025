package frc.robot.subsystems.catcher;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.catcher.CatcherConstants.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Catcher extends SubsystemBase {
    public enum CatcherArmPositions {
        TROUGH,
        CORAL_STATION
    }

    private final SparkFlex m_armMotor = new SparkFlex(ARM_ID, MotorType.kBrushless);
    private final SparkFlex m_flywheelsMotor = new SparkFlex(WHEELS_ID, MotorType.kBrushless);

    private final SparkClosedLoopController m_armController = m_armMotor.getClosedLoopController();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Catcher");
    private final DoubleEntry flywheelSpeedEntry =
            table.getDoubleTopic("Flywheel Speed").getEntry(1);

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Catcher() {
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(true);
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

        SparkFlexConfig flywheelsConfig = new SparkFlexConfig();
        flywheelsConfig
                .smartCurrentLimit(CURRENT_LIMIT)
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        m_flywheelsMotor.configure(flywheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // You need to publish a value for the entry to appear in NetworkTables
        flywheelSpeedEntry.set(1);
    }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_armMotor.stopMotor();
                    m_flywheelsMotor.stopMotor();
                })
                .andThen(Commands.idle(this));
    }

    public Command armTestCommand(DoubleSupplier armPowerSupplier) {
        return this.run(() -> {
            m_armMotor.set(armPowerSupplier.getAsDouble() * 0.25);
        });
    }

    public Command wheelTestCommand() {
        return this.run(() -> m_flywheelsMotor.set(0.5));
    }

    public Command wheelBackwardsTestCommand() {
        return this.run(() -> m_flywheelsMotor.set(-0.2));
    }

    public Command setArmPositionCommand(CatcherArmPositions armPosition) {
        return this.runOnce(() -> {
                    double position;
                    if (armPosition == CatcherArmPositions.TROUGH) {
                        position = TROUGH_POSITION;
                    } else {
                        position = CORAL_STATION_POSITION;
                    }
                    m_armController.setReference(position, ControlType.kPosition);
                })
                .andThen(Commands.idle(this));
    }
}
