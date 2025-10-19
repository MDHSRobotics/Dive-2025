package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
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
        L2,
        L3_ALGAE,
        L3,
        CURRENT_POSITION
    }

    public enum ElevatorArmPositions {
        STOWED,
        CORAL_STATION,
        L1,
        L_2_AND_3,
        ALGAE_REMOVAL,
        CURRENT_POSITION
    }
    // New arm
    private final SparkFlex m_arm2 = new SparkFlex(9, MotorType.kBrushless);
    private final RelativeEncoder m_arm2Encoder = m_arm2.getEncoder();

    // Elevator
    private final TalonFX m_elevatorMotor = new TalonFX(ELEVATOR_ID, TunerConstants.kCANBus);
    private final VoltageOut m_elevatorVoltage = new VoltageOut(0);
    private final MotionMagicExpoVoltage m_elevatorPosition = new MotionMagicExpoVoltage(ELEVATOR_MIN_LIMIT);
    private final SysIdRoutine m_elevatorRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(1).per(Second),
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> m_elevatorMotor.setControl(m_elevatorVoltage.withOutput(volts)), null, this));

    // Arm
    private final SparkFlex m_armMotor = new SparkFlex(ARM_ID, MotorType.kBrushless);
    private final AbsoluteEncoder m_armEncoder = m_armMotor.getAbsoluteEncoder();

    // private final SysIdRoutine m_armRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(1), null),
    //         new SysIdRoutine.Mechanism(m_armMotor::setVoltage, null, this));

    private final ArmFeedforward m_armFeedforward = new ArmFeedforward(ARM_K_S, ARM_K_G, ARM_K_V, ARM_K_A);
    private final TrapezoidProfile m_armProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(ARM_MAX_VELOCITY, ARM_MAX_ACCELERATION));
    private final TrapezoidProfile.State m_armGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_armStartingSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_armCurrentSetpoint = new TrapezoidProfile.State();
    private final Timer m_armProfileTimer = new Timer();
    private final SparkClosedLoopController m_armController = m_armMotor.getClosedLoopController();

    // Flywheels
    private final SparkFlex m_flywheelsMotor = new SparkFlex(WHEELS_ID, MotorType.kBrushless);

    // private final RelativeEncoder m_flywheelsEncoder = m_flywheelsMotor.getEncoder();

    // NetworkTables
    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_table = m_inst.getTable("Elevator");
    private final DoublePublisher m_armGoalPub =
            m_table.getDoubleTopic("Arm Goal Position").publish();
    private final DoublePublisher m_elevatorGoalPub =
            m_table.getDoubleTopic("Elevator Goal Position").publish();
    private final DoubleEntry m_pGainEntry =
            m_table.getDoubleTopic("Arm P Gain").getEntry(ARM_K_P, PubSubOption.excludeSelf(true));

    private double m_prevVelocity = 0;
    private final DoublePublisher m_armAccelPub =
            m_table.getDoubleTopic("Accel").publish(PubSubOption.sendAll(true));

    private final DoublePublisher m_armSetpointPositionPub =
            m_table.getDoubleTopic("Arm Setpoint Position").publish();
    private final DoublePublisher m_armSetpointVelocityPub =
            m_table.getDoubleTopic("Arm Setpoint Velocity").publish();

    private final DoublePublisher m_intakePositionPub =
            m_table.getDoubleTopic("Intake Position").publish();
    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Elevator() {
        SparkFlexConfig arm2Config = new SparkFlexConfig();
        arm2Config.smartCurrentLimit(80).idleMode(IdleMode.kBrake).inverted(true);
        arm2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // 45 degrees is 0.5905
        // Intake position is 0.22
        arm2Config
                .softLimit
                .forwardSoftLimit(0.22)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(-0.0001)
                .reverseSoftLimitEnabled(true);

        m_arm2.configure(arm2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
                        .withForwardSoftLimitEnable(true))
                .withSlot0(new Slot0Configs()
                        .withKG(ELEVATOR_K_G)
                        .withKS(ELEVATOR_K_S)
                        .withKV(ELEVATOR_K_V)
                        .withKA(ELEVATOR_K_A)
                        .withKP(ELEVATOR_K_P)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicExpo_kV(ELEVATOR_K_V)
                        .withMotionMagicExpo_kA(ELEVATOR_K_A));
        m_elevatorMotor.getConfigurator().apply(elevatorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                m_elevatorMotor.getMotorVoltage(false),
                m_elevatorMotor.getPosition(false),
                m_elevatorMotor.getVelocity(false),
                m_elevatorMotor.getClosedLoopReference(false),
                m_elevatorMotor.getClosedLoopReferenceSlope(false));

        m_elevatorMotor.optimizeBusUtilization();

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
                .inverted(false);
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

        m_pGainEntry.set(ARM_K_P);
        m_inst.addListener(m_pGainEntry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            SparkFlexConfig tempConfig = new SparkFlexConfig();
            tempConfig.closedLoop.p(event.valueData.value.getDouble());
            m_armMotor.configureAsync(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });
    }

    @Override
    public void periodic() {
        double armVelocity = m_armEncoder.getVelocity();
        double armAccel = (armVelocity - m_prevVelocity) / 0.02;
        m_armAccelPub.set(armAccel);
        m_prevVelocity = armVelocity;
        m_intakePositionPub.set(m_arm2Encoder.getPosition());
    }

    private void setElevatorPosition(ElevatorPositions elevatorPosition) {
        double position;
        if (elevatorPosition == ElevatorPositions.STOWED) {
            position = ELEVATOR_MIN_LIMIT;
        } else if (elevatorPosition == ElevatorPositions.L2) {
            position = ELEVATOR_L2_POSITION;
        } else if (elevatorPosition == ElevatorPositions.L3_ALGAE) {
            position = ELEVATOR_L3_ALGAE_POSITION;
        } else if (elevatorPosition == ElevatorPositions.L3) {
            position = ELEVATOR_L3_POSITION;
        } else if (elevatorPosition == ElevatorPositions.CURRENT_POSITION) {
            position = m_elevatorMotor.getPosition().getValueAsDouble();
        } else {
            DriverStation.reportWarning("Attempted to set the elevator to a null position!", true);
            return;
        }
        m_elevatorPosition.withPosition(position);
        m_elevatorMotor.setControl(m_elevatorPosition);
        m_elevatorGoalPub.set(position);
    }

    private void setArmPosition(ElevatorArmPositions armPosition) {
        double requestedPositionRadians;
        if (armPosition == ElevatorArmPositions.STOWED) {
            requestedPositionRadians = ARM_STOWED_POSITION;
        } else if (armPosition == ElevatorArmPositions.CORAL_STATION) {
            requestedPositionRadians = ARM_CORAL_STATION_POSITION;
        } else if (armPosition == ElevatorArmPositions.L1) {
            requestedPositionRadians = ARM_L1_POSITION;
        } else if (armPosition == ElevatorArmPositions.L_2_AND_3) {
            requestedPositionRadians = ARM_L2_AND_L3_POSITION;
        } else if (armPosition == ElevatorArmPositions.ALGAE_REMOVAL) {
            requestedPositionRadians = ARM_ALGAE_REMOVAL_POSITION;
        } else if (armPosition == ElevatorArmPositions.CURRENT_POSITION) {
            requestedPositionRadians = m_armEncoder.getPosition();
        } else {
            DriverStation.reportWarning("Attempted to set the elevator arm to a null position!", true);
            return;
        }
        m_armGoalPub.set(requestedPositionRadians);
        m_armGoal.position = requestedPositionRadians;
        m_armStartingSetpoint.position = m_armEncoder.getPosition();
        m_armStartingSetpoint.velocity = m_armEncoder.getVelocity();
        m_armProfileTimer.restart();
    }

    private void updateArmProfile() {
        double currentTime = m_armProfileTimer.get();
        TrapezoidProfile.State nextArmSetpoint = m_armProfile.calculate(currentTime, m_armStartingSetpoint, m_armGoal);
        double feedforwardVolts = m_armFeedforward.calculateWithVelocities(
                m_armEncoder.getPosition() - ARM_HORIZONTAL_OFFSET,
                m_armCurrentSetpoint.velocity,
                nextArmSetpoint.velocity);
        m_armController.setReference(
                nextArmSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts);
        m_armCurrentSetpoint = nextArmSetpoint;
        m_armSetpointPositionPub.set(m_armCurrentSetpoint.position);
        m_armSetpointVelocityPub.set(m_armCurrentSetpoint.velocity);
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

    public Command ejectCoralCommand() {
        return this.runOnce(() -> m_flywheelsMotor.set(0.5))
                .andThen(Commands.idle(this))
                .finallyDo(m_flywheelsMotor::stopMotor);
    }

    public Command ejectCoralSlowCommand() {
        return this.runOnce(() -> m_flywheelsMotor.set(0.25))
                .andThen(Commands.idle(this))
                .finallyDo(m_flywheelsMotor::stopMotor);
    }

    public Command intakeCoralCommand() {
        return this.runOnce(() -> m_flywheelsMotor.set(-0.2))
                .andThen(Commands.idle(this))
                .finallyDo(m_flywheelsMotor::stopMotor);
    }

    public Command removeAlgaeFromReefCommand(ElevatorPositions elevatorPosition) {
        return this.startRun(
                        () -> {
                            setElevatorPosition(elevatorPosition);
                            setArmPosition(ElevatorArmPositions.ALGAE_REMOVAL);
                            m_flywheelsMotor.set(0.5);
                        },
                        this::updateArmProfile)
                .finallyDo(m_flywheelsMotor::stopMotor);
    }

    public Command setElevatorPowerCommand(DoubleSupplier powerSupplier) {
        return this.run(() -> {
                    m_elevatorMotor.set(powerSupplier.getAsDouble() * 0.25);
                })
                .finallyDo(m_elevatorMotor::stopMotor);
    }

    public Command setArmPositionCommand(ElevatorArmPositions armPosition) {
        return this.startRun(() -> setArmPosition(armPosition), this::updateArmProfile);
    }

    public Command setElevatorAndArmPositionCommand(
            ElevatorPositions elevatorPosition, ElevatorArmPositions armPosition) {
        return this.startRun(
                () -> {
                    setElevatorPosition(elevatorPosition);
                    setArmPosition(armPosition);
                    if (armPosition == ElevatorArmPositions.L1 || armPosition == ElevatorArmPositions.L_2_AND_3) {
                        m_flywheelsMotor.set(-0.2);
                    }
                },
                this::updateArmProfile);
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

    public Command testIntake2Command(DoubleSupplier intake2PowerSupplier) {
        return this.run(() -> m_arm2.set(intake2PowerSupplier.getAsDouble() * 0.25))
                .finallyDo(() -> m_arm2.disable());
    }

    public void resetIntakeEncoder() {
        m_arm2Encoder.setPosition(0);
    }
}
