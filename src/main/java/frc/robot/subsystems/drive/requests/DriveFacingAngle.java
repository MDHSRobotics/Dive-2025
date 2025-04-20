package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.DriveTelemetry;

/**
 * Drives the swerve drivetrain in a field-centric manner, maintaining a
 * specified heading angle to ensure the robot is facing the desired direction.
 * <p>
 * An example scenario is that the robot is oriented to the east, the VelocityX
 * is +5 m/s, VelocityY is 0 m/s, and TargetDirection is 180 degrees.
 * In this scenario, the robot would drive northward at 5 m/s and turn clockwise
 * to a target of 180 degrees.
 * <p>
 * This swerve request is based on {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle},
 * and makes some improvements to it.
 */
public class DriveFacingAngle implements ResettableSwerveRequest {
    /**
     * The desired direction to face.
     * 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     */
    private Rotation2d targetDirection = new Rotation2d();

    /**
     * The perspective to use when determing which direction is forward for aiming.
     * By default, this swerve request considers the angle to be in <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">the blue coordinate system.</a>
     */
    private ForwardPerspectiveValue aimingPerspective = ForwardPerspectiveValue.BlueAlliance;

    private final FieldCentric fieldCentric = new FieldCentric();

    public final PhoenixPIDController headingController;
    private final double maxAngularVelocity;

    private boolean resetRequested = false;
    private boolean motionIsFinished = false;

    // NetworkTables logging
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Swerve Requests").getSubTable("Drive Facing Angle");
    public final DoublePublisher goalPositionPub =
            table.getSubTable("Goal").getDoubleTopic("Position (radians)").publish();
    public final DoublePublisher appliedVelocityPub =
            table.getDoubleTopic("Applied Velocity (rads per sec)").publish();
    public final BooleanPublisher motionIsFinishedPub =
            table.getBooleanTopic("Motion is Finished").publish();

    /**
     * Creates a new request with the given gains.
     *
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     */
    public DriveFacingAngle(double kRotationP, double maxAngularVelocity) {
        headingController = new PhoenixPIDController(kRotationP, 0.0, 0.0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.maxAngularVelocity = maxAngularVelocity;
    }

    /**
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Rotation2d fakeTargetDirection = targetDirection;

        if (resetRequested) {
            headingController.reset();
            this.resetRequested = false;
        }

        /* If the user requested a target direction according to the operator perspective, rotate our target direction by the angle */
        if (aimingPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            fakeTargetDirection = targetDirection.rotateBy(parameters.operatorForwardDirection);
        }

        double toApplyOmega = headingController.calculate(
                parameters.currentPose.getRotation().getRadians(),
                fakeTargetDirection.getRadians(),
                parameters.timestamp);

        if (maxAngularVelocity > 0.0) {
            if (toApplyOmega > maxAngularVelocity) {
                toApplyOmega = maxAngularVelocity;
            } else if (toApplyOmega < -maxAngularVelocity) {
                toApplyOmega = -maxAngularVelocity;
            }
        }

        if (headingController.atSetpoint()) {
            toApplyOmega = 0;
        }

        this.motionIsFinished = headingController.atSetpoint();

        // NetworkTables logging
        long timestamp = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);

        goalPositionPub.set(targetDirection.getRadians(), timestamp);
        appliedVelocityPub.set(toApplyOmega, timestamp);
        motionIsFinishedPub.set(motionIsFinished, timestamp);

        return fieldCentric.withRotationalRate(toApplyOmega).apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset the profile used for the target direction next time it is used.
     */
    public void resetProfile() {
        this.resetRequested = true;
        this.motionIsFinished = false;
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
    public DriveFacingAngle withVelocityX(double newVelocityX) {
        this.fieldCentric.withVelocityX(newVelocityX);
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
    public DriveFacingAngle withVelocityX(LinearVelocity newVelocityX) {
        this.fieldCentric.withVelocityX(newVelocityX);
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
    public DriveFacingAngle withVelocityY(double newVelocityY) {
        this.fieldCentric.withVelocityY(newVelocityY);
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
    public DriveFacingAngle withVelocityY(LinearVelocity newVelocityY) {
        this.fieldCentric.withVelocityY(newVelocityY);
        return this;
    }

    /**
     * Modifies the TargetDirection parameter and returns itself.
     * <p>
     * The desired direction to face. 0 Degrees is defined as in the direction of
     * the X axis. As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     *
     * @param newTargetDirection Parameter to modify
     * @return this object
     */
    public DriveFacingAngle withTargetDirection(Rotation2d newTargetDirection) {
        this.targetDirection = newTargetDirection;
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
    public DriveFacingAngle withDeadband(double newDeadband) {
        this.fieldCentric.withDeadband(newDeadband);
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
    public DriveFacingAngle withDeadband(LinearVelocity newDeadband) {
        this.fieldCentric.withDeadband(newDeadband);
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
    public DriveFacingAngle withCenterOfRotation(Translation2d newCenterOfRotation) {
        this.fieldCentric.withCenterOfRotation(newCenterOfRotation);
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
    public DriveFacingAngle withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        this.fieldCentric.withDriveRequestType(newDriveRequestType);
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
    public DriveFacingAngle withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        this.fieldCentric.withSteerRequestType(newSteerRequestType);
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
    public DriveFacingAngle withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.fieldCentric.withDesaturateWheelSpeeds(newDesaturateWheelSpeeds);
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
    public DriveFacingAngle withDrivingPerspective(ForwardPerspectiveValue newDrivingPerspective) {
        this.fieldCentric.withForwardPerspective(newDrivingPerspective);
        return this;
    }

    /**
     * Modifies the ForwardPerspective parameter and returns itself.
     * <p>
     * The perspective to use when determining which direction is forward for aiming.
     *
     * @param newAimingPerspective Parameter to modify
     * @return this object
     */
    public DriveFacingAngle withAimingPerspective(ForwardPerspectiveValue newAimingPerspective) {
        this.aimingPerspective = newAimingPerspective;
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
    public DriveFacingAngle withPIDGains(double kp, double ki, double kd) {
        this.headingController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public DriveFacingAngle withTolerance(Angle toleranceAmount) {
        this.headingController.setTolerance(toleranceAmount.in(Radians));
        return this;
    }

    /**
     * @return Whether or not the robot has reached its target rotation,
     * based on the tolerance set using {@link frc.robot.subsystems.drive.requests.DriveFacingAngle#withTolerance(Angle) withTolerance()}
     */
    public boolean motionIsFinished() {
        return this.motionIsFinished;
    }
}
