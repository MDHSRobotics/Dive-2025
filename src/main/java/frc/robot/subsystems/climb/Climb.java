// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
    public enum HookPositions {
        AWAY,
        UP,
        ENGAGED
    }

    private final SparkFlex m_backHookMotor = new SparkFlex(BACK_ID, MotorType.kBrushless);
    private final SparkFlex m_frontHookMotor = new SparkFlex(FRONT_ID, MotorType.kBrushless);

    private final DigitalInput m_cageBeamBreak = new DigitalInput(CAGE_BEAM_BREAK_DIO_CHANNEL);

    // private final SparkAbsoluteEncoder m_backEncoder = m_backHookMotor.getAbsoluteEncoder();
    // private final SparkAbsoluteEncoder m_frontEncoder = m_frontHookMotor.getAbsoluteEncoder();

    // private final SysIdRoutine m_backRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(2), null),
    //         new SysIdRoutine.Mechanism(m_backHookMotor::setVoltage, null, this));
    // private final SysIdRoutine m_frontRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_frontHookMotor::setVoltage, null, this));

    // private final SimpleMotorFeedforward m_hookFeedforward = new SimpleMotorFeedforward(K_S, K_V, K_A);

    // // Two motion profiles are needed because trapezoid profiles have their own internal state.
    // private final TrapezoidProfile m_backProfile = new TrapezoidProfile(ANGULAR_MOTION_CONSTRAINTS);
    // private TrapezoidProfile.State m_backPreviousSetpoint = new TrapezoidProfile.State();

    // private final TrapezoidProfile m_frontProfile = new TrapezoidProfile(ANGULAR_MOTION_CONSTRAINTS);
    // private TrapezoidProfile.State m_frontPreviousSetpoint = new TrapezoidProfile.State();

    // private final TrapezoidProfile.State m_backGoal = new TrapezoidProfile.State();
    // private final TrapezoidProfile.State m_frontGoal = new TrapezoidProfile.State();

    // private final SparkClosedLoopController m_backController = m_backHookMotor.getClosedLoopController();
    // private final SparkClosedLoopController m_frontController = m_frontHookMotor.getClosedLoopController();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Climb");
    private final BooleanPublisher cageBeamBroken =
            table.getBooleanTopic("Beam Broken").publish();
    // private final DoublePublisher targetPositionPub =
    //         table.getDoubleTopic("Target Position (radians)").publish();
    // private final DoubleEntry pGainEntry =
    //         table.getDoubleTopic("Hook P Gain").getEntry(K_P, PubSubOption.excludeSelf(true));

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Climb() {
        SparkFlexConfig backConfig = new SparkFlexConfig();
        backConfig.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake);
        backConfig
                .absoluteEncoder
                .zeroOffset(BACK_ZERO_OFFSET)
                .positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        backConfig
                .softLimit
                .forwardSoftLimit(BACK_MAX_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(BACK_MIN_LIMIT)
                .reverseSoftLimitEnabled(true);
        backConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        backConfig
                .signals
                .absoluteEncoderPositionPeriodMs(10)
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(10)
                .absoluteEncoderVelocityAlwaysOn(true);
        m_backHookMotor.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig frontConfig = new SparkFlexConfig();
        frontConfig.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake);
        frontConfig
                .absoluteEncoder
                .zeroOffset(FRONT_ZERO_OFFSET)
                .positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR)
                .inverted(true);
        frontConfig
                .softLimit
                .forwardSoftLimit(FRONT_MAX_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(FRONT_MIN_LIMIT)
                .reverseSoftLimitEnabled(true);
        frontConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        frontConfig
                .signals
                .absoluteEncoderPositionPeriodMs(10)
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(10)
                .absoluteEncoderVelocityAlwaysOn(true);
        m_frontHookMotor.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // pGainEntry.set(K_P);
        // inst.addListener(pGainEntry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
        //     SparkFlexConfig tempConfig = new SparkFlexConfig();
        //     tempConfig.closedLoop.p(event.valueData.value.getDouble());
        //     m_backHookMotor.configureAsync(
        //             tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //     m_frontHookMotor.configureAsync(
        //             tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // });
    }

    @Override
    public void periodic() {
        cageBeamBroken.set(!m_cageBeamBreak.get());
    }

    // private void resetProfile(HookPositions hookPositions) {
    //     if (hookPositions == HookPositions.AWAY) {
    //         m_backGoal.position = BACK_MAX_LIMIT;
    //         m_frontGoal.position = FRONT_MAX_LIMIT;
    //     } else if (hookPositions == HookPositions.UP) {
    //         m_backGoal.position = BACK_UP_POSITION;
    //         m_frontGoal.position = FRONT_UP_POSITION;
    //     } else {
    //         m_backGoal.position = BACK_MIN_LIMIT;
    //         m_frontGoal.position = FRONT_MIN_LIMIT;
    //     }
    //     m_backPreviousSetpoint.position = m_backEncoder.getPosition();
    //     m_backPreviousSetpoint.velocity = m_backEncoder.getVelocity();
    //     m_frontPreviousSetpoint.position = m_frontEncoder.getPosition();
    //     m_frontPreviousSetpoint.velocity = m_frontEncoder.getVelocity();
    // }

    // private void runProfile() {
    //     // Find the next position in the motion
    //     TrapezoidProfile.State nextBackSetpoint = m_backProfile.calculate(K_DT, m_backPreviousSetpoint, m_backGoal);
    //     TrapezoidProfile.State nextFrontSetpoint = m_frontProfile.calculate(K_DT, m_frontPreviousSetpoint,
    // m_frontGoal);
    //     // Estimate the volts required to reach the position
    //     double backFeedforwardVolts =
    //             m_hookFeedforward.calculateWithVelocities(m_backPreviousSetpoint.velocity,
    // nextBackSetpoint.velocity);
    //     double frontFeedforwardVolts =
    //             m_hookFeedforward.calculateWithVelocities(m_frontPreviousSetpoint.velocity,
    // nextFrontSetpoint.velocity);
    //     // Run the PID controller along with the estimated volts
    //     m_backController.setReference(
    //             nextBackSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, backFeedforwardVolts);
    //     m_frontController.setReference(
    //             nextFrontSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, frontFeedforwardVolts);
    //     // Update current setpoint
    //     m_backPreviousSetpoint = nextBackSetpoint;
    //     m_frontPreviousSetpoint = nextFrontSetpoint;
    // }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_frontHookMotor.stopMotor();
                    m_backHookMotor.stopMotor();
                })
                .andThen(Commands.idle(this));
    }

    public Command motorTestCommand(DoubleSupplier backMotorPowerSupplier, DoubleSupplier frontMotorPowerSupplier) {
        return this.run(() -> {
            m_backHookMotor.set(backMotorPowerSupplier.getAsDouble() * 0.5);
            m_frontHookMotor.set(frontMotorPowerSupplier.getAsDouble() * 0.5);
        });
    }

    // public Command setHookPositionCommand(HookPositions hookPositions) {
    //     return this.runOnce(() -> {
    //         double backPosition;
    //         double frontPosition;
    //         if (hookPositions == HookPositions.AWAY) {
    //             backPosition = BACK_MAX_LIMIT;
    //             frontPosition = FRONT_MAX_LIMIT;
    //         } else if (hookPositions == HookPositions.UP) {
    //             backPosition = BACK_UP_POSITION;
    //             frontPosition = FRONT_UP_POSITION;
    //         } else {
    //             backPosition = BACK_MIN_LIMIT;
    //             frontPosition = FRONT_MIN_LIMIT;
    //         }
    //         System.out.println(m_backController.setReference(backPosition, ControlType.kPosition));
    //         System.out.println(m_frontController.setReference(frontPosition, ControlType.kPosition));
    //     });
    // }

    // public Command setHookPositionCommand(HookPositions hookPositions) {
    //     return this.startRun(() -> this.resetProfile(hookPositions), this::runProfile);
    // }

    // public Command backSysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_backRoutine.quasistatic(direction);
    // }

    // public Command backSysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_backRoutine.dynamic(direction);
    // }
}
