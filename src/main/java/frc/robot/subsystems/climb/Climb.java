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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
    private final SparkFlex m_backHookMotor = new SparkFlex(BACK_ID, MotorType.kBrushless);
    private final SparkFlex m_frontHookMotor = new SparkFlex(FRONT_ID, MotorType.kBrushless);

    private final SysIdRoutine m_backRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_backHookMotor::setVoltage, null, this));
    private final SysIdRoutine m_frontRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_frontHookMotor::setVoltage, null, this));

    private final SimpleMotorFeedforward m_hookFeedforward = new SimpleMotorFeedforward(K_S, K_V, K_A);

    // Two motion profiles are needed because trapezoid profiles have their own internal state.
    private final TrapezoidProfile m_backProfile = new TrapezoidProfile(ANGULAR_MOTION_CONSTRAINTS);
    private final TrapezoidProfile m_frontProfile = new TrapezoidProfile(ANGULAR_MOTION_CONSTRAINTS);

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Climb() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(true);
        config.absoluteEncoder
                .positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR)
                .averageDepth(2);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).p(K_P).d(K_D);
        config.signals
                .absoluteEncoderPositionPeriodMs(10)
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(10)
                .absoluteEncoderVelocityAlwaysOn(true);
        m_backHookMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_frontHookMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

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

    public Command leftSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_backRoutine.quasistatic(direction);
    }

    public Command leftSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_backRoutine.dynamic(direction);
    }

    public Command rightSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_frontRoutine.quasistatic(direction);
    }

    public Command rightSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_frontRoutine.dynamic(direction);
    }
}
