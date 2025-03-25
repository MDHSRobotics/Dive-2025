package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.TunerConstants;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
    public enum ElevatorArmPositions {
        STOWED,
        CORAL_STATION,
        L1,
        L_2_AND_3,
        L4
    }

    private final TalonFX m_elevatorMotor = new TalonFX(ELEVATOR_ID, TunerConstants.kCANBus);
    // private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //                 Volts.of(0.1).per(Second),
    //                 Volts.of(4),
    //                 null,
    //                 (state) -> SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //                 (volts) -> m_elevatorMotor.setControl(m_elevatorVoltage.withOutput(volts)), null, this));

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
    public Elevator() {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        elevatorConfig
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ELEVATOR_SENSOR_TO_MECHANISM_RATIO));
        m_elevatorMotor.getConfigurator().apply(elevatorConfig);

        // SparkFlexConfig elevatorConfig = new SparkFlexConfig();
        // elevatorConfig.smartCurrentLimit(80).idleMode(IdleMode.kBrake);
        // elevatorConfig
        //         .signals
        //         .primaryEncoderPositionPeriodMs(10)
        //         .primaryEncoderPositionAlwaysOn(true)
        //         .primaryEncoderVelocityPeriodMs(10)
        //         .primaryEncoderVelocityAlwaysOn(true);
        // m_elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
                    m_elevatorMotor.stopMotor();
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

    public Command setArmPositionCommand(ElevatorArmPositions armPosition) {
        return this.runOnce(() -> {
            double position;
            if (armPosition == ElevatorArmPositions.STOWED) {
                position = ARM_MIN_LIMIT;
            } else if (armPosition == ElevatorArmPositions.CORAL_STATION) {
                position = CORAL_STATION_POSITION;
            } else if (armPosition == ElevatorArmPositions.L1) {
                position = L1_POSITION;
            } else if (armPosition == ElevatorArmPositions.L_2_AND_3) {
                position = L2_AND_L3_POSITION;
            } else if (armPosition == ElevatorArmPositions.L4) {
                position = L4_POSITION;
            } else {
                DriverStation.reportWarning("Attempted to set the elevator arm to a null position!", true);
                return;
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
        return setArmPositionCommand(ElevatorArmPositions.CORAL_STATION).andThen(runWheelCommand());
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

    public Command setArmPositionAndWaitCommand(ElevatorArmPositions armPosition) {
        return setArmPositionCommand(armPosition).andThen(Commands.idle(this));
    }

    public Command setElevatorPowerCommand(DoubleSupplier powerSupplier) {
        return this.run(() -> {
                    m_elevatorMotor.set(powerSupplier.getAsDouble() * 0.25);
                })
                .finallyDo(m_elevatorMotor::stopMotor);
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.dynamic(direction);
    // }
}
