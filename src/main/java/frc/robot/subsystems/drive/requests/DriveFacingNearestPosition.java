package frc.robot.subsystems.drive.requests;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import java.util.List;

/**
 * Drives the swerve drivetrain in a field-centric manner, maintaining a
 * specified heading angle to ensure the robot is facing the nearest of the given positions.
 * <p>
 * It also has gives option of using PathPlanner's {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator SwerveSetpointGenerator}
 * to ensure that the motion respects the robot's contraints.
 * <p>
 * An example scenario is that the robot is at (0,0),
 * and the user provides a list of (1,1) and (-50,-50).
 * In this scenario, the robot would face (1,1).
 * <p>
 * This swerve request is based on {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle},
 * and makes some improvements to it.
 */
public class DriveFacingNearestPosition implements ResettableSwerveRequest {
    /**
     * The desired positions to face.
     */
    private List<Translation2d> m_targetPositions = List.of(new Translation2d());

    private final DriveFacingPosition m_driveFacingPosition;

    /**
     * Creates a new request with the given gains, and without Swerve Setpoint Generation.
     *
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     */
    public DriveFacingNearestPosition(double kRotationP, double maxAngularVelocity) {
        m_driveFacingPosition = new DriveFacingPosition(kRotationP, maxAngularVelocity);
    }

    /**
     * Creates a new request with the given gains, and with Swerve Setpoint Generation.
     *
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     * @param swerveSetpointGenerator The Swerve Setpoint Generator to use when driving.
     * @param updatePeriod The amount of time between robot updates in seconds.
     */
    public DriveFacingNearestPosition(
            double kRotationP,
            double maxAngularVelocity,
            SwerveSetpointGenerator swerveSetpointGenerator,
            double updatePeriod) {
        m_driveFacingPosition =
                new DriveFacingPosition(kRotationP, maxAngularVelocity, swerveSetpointGenerator, updatePeriod);
    }

    /**
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Translation2d targetPosition = parameters.currentPose.getTranslation().nearest(m_targetPositions);

        return m_driveFacingPosition.withTargetPosition(targetPosition).apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset the profile used for the target direction next time it is used.
     */
    public void resetRequest() {
        m_driveFacingPosition.resetRequest();
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     * <p>
     * The velocity in the X direction, in m/s. X is defined as forward according to
     * WPILib convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withVelocityX(double newVelocityX) {
        m_driveFacingPosition.withVelocityX(newVelocityX);
        return this;
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     * <p>
     * The velocity in the X direction, in m/s. X is defined as forward according to
     * WPILib convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withVelocityX(LinearVelocity newVelocityX) {
        m_driveFacingPosition.withVelocityX(newVelocityX);
        return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     * <p>
     * The velocity in the Y direction, in m/s. Y is defined as to the left
     * according to WPILib convention, so this determines how fast to travel to the
     * left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withVelocityY(double newVelocityY) {
        m_driveFacingPosition.withVelocityY(newVelocityY);
        return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     * <p>
     * The velocity in the Y direction, in m/s. Y is defined as to the left
     * according to WPILib convention, so this determines how fast to travel to the
     * left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withVelocityY(LinearVelocity newVelocityY) {
        m_driveFacingPosition.withVelocityY(newVelocityY);
        return this;
    }

    /**
     * Modifies the targetPositions parameter and returns itself.
     * <p>
     * The desired positions to face.
     * The origin for this position should be <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">the blue alliance.</a>
     *
     * @param newTargetPositions Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withTargetPositions(List<Translation2d> newTargetPositions) {
        m_targetPositions = newTargetPositions;
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withDeadband(double newDeadband) {
        m_driveFacingPosition.withDeadband(newDeadband);
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withDeadband(LinearVelocity newDeadband) {
        m_driveFacingPosition.withDeadband(newDeadband);
        return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     * <p>
     * The center of rotation the robot should rotate around. This is (0,0) by
     * default, which will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withCenterOfRotation(Translation2d newCenterOfRotation) {
        m_driveFacingPosition.withCenterOfRotation(newCenterOfRotation);
        return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        m_driveFacingPosition.withDriveRequestType(newDriveRequestType);
        return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        m_driveFacingPosition.withSteerRequestType(newSteerRequestType);
        return this;
    }

    /**
     * Modifies the ForwardPerspective parameter and returns itself.
     * <p>
     * The perspective to use when determining which direction is forward for driving.
     *
     * @param newDrivingPerspective Parameter to modify
     * @return this object
     */
    public DriveFacingNearestPosition withDrivingPerspective(ForwardPerspectiveValue newDrivingPerspective) {
        m_driveFacingPosition.withDrivingPerspective(newDrivingPerspective);
        return this;
    }

    /**
     * Modifies the PID gains for the heading controller and returns itself.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @return this object
     */
    public DriveFacingNearestPosition withPIDGains(double kp, double ki, double kd) {
        m_driveFacingPosition.withPIDGains(kp, ki, kd);
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public DriveFacingNearestPosition withTolerance(Angle toleranceAmount) {
        m_driveFacingPosition.withTolerance(toleranceAmount);
        return this;
    }

    /**
     * @return Whether or not the robot has reached its target rotation,
     * based on the tolerance set using {@link frc.robot.subsystems.drive.requests.DriveFacingNearestPosition#withTolerance(Angle) withTolerance()}
     */
    public boolean motionIsFinished() {
        return m_driveFacingPosition.motionIsFinished();
    }
}
