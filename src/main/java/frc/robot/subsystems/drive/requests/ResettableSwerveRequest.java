package frc.robot.subsystems.drive.requests;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * This interface is implemented by our custom swerve requests.
 * This allows us to reset them without having to know which request they are.
 */
public interface ResettableSwerveRequest extends SwerveRequest {
    void resetRequest();
}
