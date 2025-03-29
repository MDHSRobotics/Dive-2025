package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.TunerConstants;
import java.util.EnumSet;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
    public enum ElevatorPositions {
        STOWED,
        CORAL_STATION,
        L1,
        L2,
        L3,
        L4
    }

    public enum ElevatorArmPositions {
        STOWED,
        CORAL_STATION,
        L1,
        L_2_AND_3,
        L4
    }

    private final TalonFX m_elevatorMotor = new TalonFX(ELEVATOR_ID, TunerConstants.kCANBus);
    private final VoltageOut m_elevatorVoltage = new VoltageOut(0);
    private final SysIdRoutine m_elevatorRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.05).per(Second),
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> m_elevatorMotor.setControl(m_elevatorVoltage.withOutput(volts)), null, this));

    private final SparkFlex m_armMotor = new SparkFlex(ARM_ID, MotorType.kBrushless);
    private final AbsoluteEncoder m_armEncoder = m_armMotor.getAbsoluteEncoder();

    private final SysIdRoutine m_armRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(1), null),
            new SysIdRoutine.Mechanism(m_armMotor::setVoltage, null, this));

    private final ArmFeedforward m_armFeedforward = new ArmFeedforward(ARM_K_S, ARM_K_G, ARM_K_V, ARM_K_A);
    private final TrapezoidProfile m_armProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(ARM_MAX_VELOCITY, ARM_MAX_ACCELERATION));
    private final TrapezoidProfile.State m_armGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_armStartingSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_armCurrentSetpoint = new TrapezoidProfile.State();
    private final Timer m_armProfileTimer = new Timer();
    private double m_armPosition;

    private final SparkFlex m_flywheelsMotor = new SparkFlex(WHEELS_ID, MotorType.kBrushless);

    private final RelativeEncoder m_flywheelsEncoder = m_flywheelsMotor.getEncoder();

    private final SparkClosedLoopController m_armController = m_armMotor.getClosedLoopController();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Elevator");
    private final DoubleEntry flywheelSpeedEntry =
            table.getDoubleTopic("Flywheel Speed").getEntry(1);
    private final DoublePublisher targetPositionPub =
            table.getDoubleTopic("Target Position (radians)").publish();
    private final DoubleEntry pGainEntry =
            table.getDoubleTopic("Arm P Gain").getEntry(ARM_K_P, PubSubOption.excludeSelf(true));

    private double m_prevVelocity = 0;
    private final DoublePublisher armAccelPub = table.getDoubleTopic("Accel").publish(PubSubOption.sendAll(true));

    private final DoublePublisher currentPositionSetpointPub =
            table.getDoubleTopic("Current Setpoint Position").publish();
    private final DoublePublisher currentVelocitySetpointPub =
            table.getDoubleTopic("Current Setpoint Velocity").publish();

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Elevator() {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        elevatorConfig
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ELEVATOR_SENSOR_TO_MECHANISM_RATIO))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withReverseSoftLimitThreshold(ELEVATOR_MIN_LIMIT)
                        .withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ELEVATOR_MAX_LIMIT)
                        .withForwardSoftLimitEnable(true));
        m_elevatorMotor.getConfigurator().apply(elevatorConfig);

        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake);
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
                .zeroOffset(ARM_ZERO_OFFSET)
                .inverted(true);
        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).p(ARM_K_P);
        armConfig.signals.absoluteEncoderPositionAlwaysOn(true).absoluteEncoderVelocityAlwaysOn(true);
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

        pGainEntry.set(ARM_K_P);
        inst.addListener(pGainEntry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            SparkFlexConfig tempConfig = new SparkFlexConfig();
            tempConfig.closedLoop.p(event.valueData.value.getDouble());
            m_armMotor.configureAsync(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });
    }

    @Override
    public void periodic() {
        double armVelocity = m_armEncoder.getVelocity();
        double armAccel = (armVelocity - m_prevVelocity) / 0.02;
        armAccelPub.set(armAccel);
        m_prevVelocity = armVelocity;
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
                    m_armMotor.set(armPowerSupplier.getAsDouble() * 0.5);
                })
                .finallyDo(m_armMotor::stopMotor);
    }

    public Command profiledSetArmPositionCommand(ElevatorArmPositions armPosition) {
        return this.startRun(
                () -> {
                    if (armPosition == ElevatorArmPositions.STOWED) {
                        m_armPosition = ARM_MIN_LIMIT;
                    } else if (armPosition == ElevatorArmPositions.CORAL_STATION) {
                        m_armPosition = CORAL_STATION_POSITION;
                    } else if (armPosition == ElevatorArmPositions.L1) {
                        m_armPosition = L1_POSITION;
                    } else if (armPosition == ElevatorArmPositions.L_2_AND_3) {
                        m_armPosition = L2_AND_L3_POSITION;
                    } else if (armPosition == ElevatorArmPositions.L4) {
                        m_armPosition = L4_POSITION;
                    } else {
                        DriverStation.reportWarning("Attempted to set the elevator arm to a null position!", true);
                        return;
                    }
                    targetPositionPub.set(m_armPosition);
                    m_armGoal.position = m_armPosition;
                    m_armStartingSetpoint.position = m_armEncoder.getPosition();
                    m_armStartingSetpoint.velocity = m_armEncoder.getVelocity();
                    m_armProfileTimer.restart();
                },
                () -> {
                    double currentTime = m_armProfileTimer.get();
                    TrapezoidProfile.State nextArmSetpoint =
                            m_armProfile.calculate(currentTime, m_armStartingSetpoint, m_armGoal);
                    double feedforwardVolts = m_armFeedforward.calculateWithVelocities(
                            m_armEncoder.getPosition() - ARM_HORIZONTAL_OFFSET,
                            m_armCurrentSetpoint.velocity,
                            nextArmSetpoint.velocity);
                    m_armController.setReference(
                            nextArmSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts);
                    m_armCurrentSetpoint = nextArmSetpoint;
                    currentPositionSetpointPub.set(m_armCurrentSetpoint.position);
                    currentVelocitySetpointPub.set(m_armCurrentSetpoint.velocity);
                });
    }

    public Command runWheelCommand() {
        return this.runOnce(() -> m_flywheelsMotor.set(0.5))
                .andThen(Commands.idle(this))
                .finallyDo(m_flywheelsMotor::stopMotor);
    }

    public Command raiseAndRunWheelCommand() {
        return profiledSetArmPositionCommand(ElevatorArmPositions.CORAL_STATION).andThen(runWheelCommand());
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

    public Command profiledSetArmPositionAndWaitCommand(ElevatorArmPositions armPosition) {
        return profiledSetArmPositionCommand(armPosition).andThen(Commands.idle(this));
    }

    public Command setElevatorPowerCommand(DoubleSupplier powerSupplier) {
        return this.run(() -> {
                    m_elevatorMotor.set(powerSupplier.getAsDouble() * 0.25);
                })
                .finallyDo(m_elevatorMotor::stopMotor);
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_elevatorRoutine.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_elevatorRoutine.dynamic(direction);
    }

    public Command elevatorTestCommand() {
        return this.runOnce(() -> m_elevatorMotor.setControl(m_elevatorVoltage.withOutput(0.01)))
                .andThen(Commands.idle(this))
                .finallyDo(m_elevatorMotor::stopMotor);
    }
}
