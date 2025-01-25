// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
    private final SparkFlex m_leftMotor = new SparkFlex(ClimbConstants.LEFT_ID, MotorType.kBrushless);
    private final SparkFlex m_rightMotor = new SparkFlex(ClimbConstants.RIGHT_ID, MotorType.kBrushless);

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Climb() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(ClimbConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(ClimbConstants.ENOCDER_CONVERSION_FACTOR)
                .velocityConversionFactor(ClimbConstants.ENOCDER_CONVERSION_FACTOR);
        config.closedLoop.p(ClimbConstants.K_P).d(ClimbConstants.K_D).positionWrappingInputRange(0, 1);
        config.closedLoop
                .maxMotion
                .maxVelocity(ClimbConstants.MAX_VELOCITY)
                .maxAcceleration(ClimbConstants.MAX_ACCELERATION);
        m_leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command motorTestCommand(DoubleSupplier leftMotorPowerSupplier, DoubleSupplier rightMotorPowerSupplier) {
        return this.run(() -> {
            m_leftMotor.set(leftMotorPowerSupplier.getAsDouble());
            m_rightMotor.set(rightMotorPowerSupplier.getAsDouble());
        });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
