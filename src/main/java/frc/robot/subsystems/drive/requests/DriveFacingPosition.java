package frc.robot.subsystems.drive.requests;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Drives the swerve drivetrain in a field-centric manner, maintaining a
 * specified heading angle to ensure the robot is facing the desired position.
 * <p>
 * An example scenario is that the robot is at (0,0),
 * and the target position is at (1,1).
 * In this scenario, the robot would rotate to a heading of 45 degrees counterclockwise to face the target position.
 * <p>
 * This swerve request is based on {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle},
 * and makes some improvements to it.
 */
public class DriveFacingPosition implements ResettableSwerveRequest {
    /**
     * The desired position to face.
     */
    private Translation2d m_targetPosition = new Translation2d();

    private final DriveFacingAngle m_driveFacingAngle;

    /**
     * Creates a new request with the given gains.
     *
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     */
    public DriveFacingPosition(double kRotationP, double maxAngularVelocity) {
        m_driveFacingAngle = new DriveFacingAngle(kRotationP, maxAngularVelocity);
    }

    /**
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        // Find the angle of the vector that the goal would make if the robot was the origin
        double xDistance = m_targetPosition.getX() - parameters.currentPose.getX();
        double yDistance = m_targetPosition.getY() - parameters.currentPose.getY();
        double yawRadians = Math.atan2(yDistance, xDistance);
        Rotation2d targetDirection = Rotation2d.fromRadians(yawRadians);

        return m_driveFacingAngle.withTargetDirection(targetDirection).apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset the profile used for the target direction next time it is used.
     */
    public void resetRequest() {
        m_driveFacingAngle.resetRequest();
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
    public DriveFacingPosition withVelocityX(double newVelocityX) {
        m_driveFacingAngle.withVelocityX(newVelocityX);
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
    public DriveFacingPosition withVelocityX(LinearVelocity newVelocityX) {
        m_driveFacingAngle.withVelocityX(newVelocityX);
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
    public DriveFacingPosition withVelocityY(double newVelocityY) {
        m_driveFacingAngle.withVelocityY(newVelocityY);
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
    public DriveFacingPosition withVelocityY(LinearVelocity newVelocityY) {
        m_driveFacingAngle.withVelocityY(newVelocityY);
        return this;
    }

    /**
     * Modifies the targetPosition parameter and returns itself.
     * <p>
     * The desired position to face.
     * The origin for this position should be <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">the blue alliance.</a>
     *
     * @param newTargetPosition Parameter to modify
     * @return this object
     */
    public DriveFacingPosition withTargetPosition(Translation2d newTargetPosition) {
        m_targetPosition = newTargetPosition;
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
    public DriveFacingPosition withDeadband(double newDeadband) {
        m_driveFacingAngle.withDeadband(newDeadband);
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
    public DriveFacingPosition withDeadband(LinearVelocity newDeadband) {
        m_driveFacingAngle.withDeadband(newDeadband);
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
    public DriveFacingPosition withCenterOfRotation(Translation2d newCenterOfRotation) {
        m_driveFacingAngle.withCenterOfRotation(newCenterOfRotation);
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
    public DriveFacingPosition withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        m_driveFacingAngle.withDriveRequestType(newDriveRequestType);
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
    public DriveFacingPosition withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        m_driveFacingAngle.withSteerRequestType(newSteerRequestType);
        return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     * <p>
     * Whether to desaturate wheel speeds before applying. For more information, see
     * the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public DriveFacingPosition withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        m_driveFacingAngle.withDesaturateWheelSpeeds(newDesaturateWheelSpeeds);
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
    public DriveFacingPosition withDrivingPerspective(ForwardPerspectiveValue newDrivingPerspective) {
        m_driveFacingAngle.withDrivingPerspective(newDrivingPerspective);
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
    public DriveFacingPosition withPIDGains(double kp, double ki, double kd) {
        m_driveFacingAngle.withPIDGains(kp, ki, kd);
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public DriveFacingPosition withTolerance(Angle toleranceAmount) {
        m_driveFacingAngle.withTolerance(toleranceAmount);
        return this;
    }

    /**
     * @return Whether or not the robot has reached its target rotation,
     * based on the tolerance set using {@link frc.robot.subsystems.drive.requests.DriveFacingPosition#withTolerance(Angle) withTolerance()}
     */
    public boolean motionIsFinished() {
        return m_driveFacingAngle.motionIsFinished();
    }
}
