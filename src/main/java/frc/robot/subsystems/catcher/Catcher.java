package frc.robot.subsystems.catcher;

<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/catcher/Catcher.java
import static frc.robot.subsystems.catcher.CatcherConstants.*;
========
import static frc.robot.subsystems.catcher.ElevatorConstants.*;
>>>>>>>> parent of 549f9b7 (Added more constants and fixed some commands):src/main/java/frc/robot/subsystems/catcher/Elevator.java

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Catcher extends SubsystemBase {
    public enum CatcherArmPositions {
        STOWED,
        TROUGH,
        CORAL_STATION,
        UP,
        L_2
    }

<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/catcher/Catcher.java
========
    private final TalonFX m_elevatorMotor = new TalonFX(15);

>>>>>>>> parent of 549f9b7 (Added more constants and fixed some commands):src/main/java/frc/robot/subsystems/catcher/Elevator.java
    private final SparkFlex m_armMotor = new SparkFlex(ARM_ID, MotorType.kBrushless);
    private final SparkFlex m_flywheelsMotor = new SparkFlex(WHEELS_ID, MotorType.kBrushless);

    private final RelativeEncoder m_flywheelsEncoder = m_flywheelsMotor.getEncoder();

    private final SparkClosedLoopController m_armController = m_armMotor.getClosedLoopController();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Catcher");
    private final DoubleEntry flywheelSpeedEntry =
            table.getDoubleTopic("Flywheel Speed").getEntry(1);
    private final DoublePublisher targetPositionPub =
            table.getDoubleTopic("Target Position (radians)").publish();
    // private final DoubleEntry pGainEntry =
    //         table.getDoubleTopic("Arm P Gain").getEntry(K_P, PubSubOption.excludeSelf(true));

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
        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).p(K_P);
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
        flywheelsConfig.signals.primaryEncoderVelocityPeriodMs(10).primaryEncoderVelocityAlwaysOn(true);
        m_flywheelsMotor.configure(flywheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // You need to publish a value for the entry to appear in NetworkTables
        flywheelSpeedEntry.set(1);

        // pGainEntry.set(K_P);
        // inst.addListener(pGainEntry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
        //     SparkFlexConfig tempConfig = new SparkFlexConfig();
        //     tempConfig.closedLoop.p(event.valueData.value.getDouble());
        //     m_armMotor.configureAsync(tempConfig, ResetMode.kNoResetSafeParameters,
        // PersistMode.kNoPersistParameters);
        // });
    }

    private boolean wheelsAreStopped() {
        return MathUtil.isNear(0, m_flywheelsEncoder.getVelocity(), 0.001);
    }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_armMotor.stopMotor();
                    m_flywheelsMotor.stopMotor();
                })
                .andThen(Commands.idle(this));
    }

    public Command setArmPowerCommand(DoubleSupplier armPowerSupplier) {
        return this.run(() -> {
                    m_armMotor.set(armPowerSupplier.getAsDouble() * 0.25);
                })
                .finallyDo(m_armMotor::stopMotor);
    }

    public Command setArmPositionAndEndCommand(CatcherArmPositions armPosition) {
        return this.runOnce(() -> {
            double position;
            if (armPosition == CatcherArmPositions.STOWED) {
                position = ARM_MIN_LIMIT;
            } else if (armPosition == CatcherArmPositions.TROUGH) {
                position = TROUGH_POSITION;
            } else if (armPosition == CatcherArmPositions.CORAL_STATION) {
                position = CORAL_STATION_POSITION;
            } else if (armPosition == CatcherArmPositions.UP) {
                position = UP_POSITION;
            } else {
                position = L2_POSITION;
            }
            m_armController.setReference(position, ControlType.kPosition);
            targetPositionPub.set(position);
        });
    }

    public Command runWheelCommand() {
        return this.runOnce(() -> m_flywheelsMotor.set(0.5))
                .andThen(Commands.idle(this))
                .finallyDo(m_flywheelsMotor::stopMotor);
    }

    public Command raiseAndRunWheelCommand() {
        return setArmPositionAndEndCommand(CatcherArmPositions.CORAL_STATION).andThen(runWheelCommand());
    }

    public Command wheelBackwardsCommand() {
        return this.run(() -> {
                    if (wheelsAreStopped()) {
                        m_flywheelsMotor.set(-1);
                    } else {
                        m_flywheelsMotor.set(-0.2);
                    }
                })
                .finallyDo(m_flywheelsMotor::stopMotor);
    }

    public Command runWheelUntilStoppedCommand() {
        return runWheelCommand().until(this::wheelsAreStopped);
    }

    public Command wheelBackwardsWhileRaisingArmCommand() {
        return this.runOnce(() -> {
                    m_flywheelsMotor.set(-0.2);
                    m_armMotor.set(-0.1);
                })
                .andThen(Commands.idle(this))
                .finallyDo(() -> {
                    m_flywheelsMotor.stopMotor();
                    m_armMotor.stopMotor();
                });
    }

    public Command setArmPositionCommand(CatcherArmPositions armPosition) {
        return setArmPositionAndEndCommand(armPosition).andThen(Commands.idle(this));
    }
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/catcher/Catcher.java
========

    public Command raiseElevatorTestCommand() {
        return this.run(() -> {
            m_elevatorMotor.set(0.5);
        });
    }

    public Command lowerElevatorCommand() {
        return this.runOnce(() -> {
            m_elevatorMotor.set(-0.5);
        });
    }
>>>>>>>> parent of 549f9b7 (Added more constants and fixed some commands):src/main/java/frc/robot/subsystems/catcher/Elevator.java
}
