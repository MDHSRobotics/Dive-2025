// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private final SparkFlex left_Climb = new SparkFlex(2, MotorType.kBrushless);
    private final SparkFlex right_Climb = new SparkFlex(3, MotorType.kBrushless);

    public Climb() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(ClimbConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(ClimbConstants.GEAR_RATIO)
                .velocityConversionFactor(ClimbConstants.GEAR_RATIO);
        config.closedLoop.p(ClimbConstants.K_P).d(ClimbConstants.K_D).positionWrappingInputRange(0, 1);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            /* one-time action goes here */
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
