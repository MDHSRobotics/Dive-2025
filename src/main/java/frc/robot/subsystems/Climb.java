// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimbConstants;
import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
    private final SparkFlex m_backMotor = new SparkFlex(ClimbConstants.BACK_ID, MotorType.kBrushless);
    private final SparkFlex m_frontMotor = new SparkFlex(ClimbConstants.FRONT_ID, MotorType.kBrushless);

    private final SysIdRoutine m_backRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_backMotor::setVoltage, null, this));
    private final SysIdRoutine m_frontRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_frontMotor::setVoltage, null, this));

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Climb() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(ClimbConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(ClimbConstants.VELOCITY_CONVERSION_FACTOR);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(ClimbConstants.K_P)
                .d(ClimbConstants.K_D);
        config.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true);
        m_backMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        m_frontMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_frontMotor.stopMotor();
                    m_backMotor.stopMotor();
                })
                .andThen(Commands.idle(this));
    }

    public Command motorTestCommand(DoubleSupplier backMotorPowerSupplier, DoubleSupplier frontMotorPowerSupplier) {
        return this.run(() -> {
            m_backMotor.set(backMotorPowerSupplier.getAsDouble());
            m_frontMotor.set(frontMotorPowerSupplier.getAsDouble());
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
