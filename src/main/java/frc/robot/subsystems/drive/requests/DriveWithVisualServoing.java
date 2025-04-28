package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.units.measure.*;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Drives the swerve drivetrain in a field-centric manner,
 * maintaining a heading angle to ensure that limelight tx is 0.
 * <p>
 * An example scenario is that the robot sees an apriltag at tx = 10 (degrees clockwise).
 * The robot would then rotate 10 degrees clockwise to face the tag.
 * <p>
 * If no tag is visible, the robot will finish rotating based on the last known tx value, and then stop rotating.
 * <p>
 * This swerve request is based on {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle},
 * and makes some improvements to it.
 * <p>
 * Note: this request makes use of a <a href="https://docs.wpilib.org/en/stable/docs/software/networktables/listening-for-change.html#using-networktableinstance-to-listen-for-changes">NetworkTable listener</a>
 * so that it can differentiate between new and old tx values,
 * even when the values are the same.
 * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-with-visual-servoing">Explanation of visual servoing</a>
 */
public class DriveWithVisualServoing implements ResettableSwerveRequest {
    /**
     * The target direction for the swerve request.
     * This field is not modifiable outside of this class because it is determined by tx.
     */
    private Rotation2d m_targetDirection = new Rotation2d();

    private final DriveFacingAngle m_driveFacingAngle;

    private boolean m_resetRequested = false;

    private final DoubleSubscriber m_txSub;
    private final AtomicReference<Double> m_txValue = new AtomicReference<Double>(0.0);

    /**
     * Creates a new profiled request with the given gains and camera.
     *
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     * @param cameraTable The NetworkTable for the limelight.
     */
    public DriveWithVisualServoing(double kRotationP, double maxAngularVelocity, NetworkTable cameraTable) {
        m_driveFacingAngle = new DriveFacingAngle(kRotationP, maxAngularVelocity);

        m_txSub = cameraTable.getDoubleTopic("tx").subscribe(0);

        cameraTable
                .getInstance()
                .addListener(
                        m_txSub,
                        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                        event -> m_txValue.set(event.valueData.value.getDouble()));
    }

    /**
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Rotation2d currentAngle = parameters.currentPose.getRotation();

        if (m_resetRequested) {
            m_targetDirection = currentAngle;
            m_txValue.set(null);
            m_resetRequested = false;
        }

        Double tx = m_txValue.getAndSet(null);
        // If tx has updated, update the target direction.
        if (tx != null) {
            // You need to subtract instead of adding because the current angle is counterclockwise, but tx is
            // clockwise.
            m_targetDirection = currentAngle.minus(Rotation2d.fromDegrees(tx));
        }

        return m_driveFacingAngle.withTargetDirection(m_targetDirection).apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset the profile used for the target direction next time it is used.
     */
    public void resetRequest() {
        m_resetRequested = true;
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
    public DriveWithVisualServoing withVelocityX(double newVelocityX) {
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
    public DriveWithVisualServoing withVelocityX(LinearVelocity newVelocityX) {
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
    public DriveWithVisualServoing withVelocityY(double newVelocityY) {
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
    public DriveWithVisualServoing withVelocityY(LinearVelocity newVelocityY) {
        m_driveFacingAngle.withVelocityY(newVelocityY);
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
    public DriveWithVisualServoing withDeadband(double newDeadband) {
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
    public DriveWithVisualServoing withDeadband(LinearVelocity newDeadband) {
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
    public DriveWithVisualServoing withCenterOfRotation(Translation2d newCenterOfRotation) {
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
    public DriveWithVisualServoing withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
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
    public DriveWithVisualServoing withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
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
    public DriveWithVisualServoing withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
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
    public DriveWithVisualServoing withDrivingPerspective(ForwardPerspectiveValue newDrivingPerspective) {
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
    public DriveWithVisualServoing withPIDGains(double kp, double ki, double kd) {
        m_driveFacingAngle.withPIDGains(kp, ki, kd);
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public DriveWithVisualServoing withTolerance(Angle toleranceAmount) {
        m_driveFacingAngle.withTolerance(toleranceAmount);
        return this;
    }

    /**
     * @return Whether or not the robot has reached its target rotation,
     * based on the tolerance set using {@link frc.robot.subsystems.drive.requests.DriveFacingAngle#withTolerance(Angle) withTolerance()}
     */
    public boolean motionIsFinished() {
        return m_driveFacingAngle.motionIsFinished();
    }
}
