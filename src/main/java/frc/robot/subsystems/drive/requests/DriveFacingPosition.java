package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.DriveTelemetry;

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
     * The velocity in the X direction, in m/s.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     */
    private double velocityX = 0;
    /**
     * The velocity in the Y direction, in m/s.
     * Y is defined as to the left according to WPILib convention,
     * so this determines how fast to travel to the left.
     */
    private double velocityY = 0;
    /**
     * The desired position to face.
     */
    private Translation2d targetPosition = new Translation2d();

    /**
     * The allowable deadband of the request, in m/s.
     */
    private double deadband = 0;
    /**
     * The center of rotation the robot should rotate around.
     * This is (0,0) by default, which will rotate around the center of the robot.
     */
    private Translation2d centerOfRotation = new Translation2d();

    /**
     * The type of control request to use for the drive motor.
     */
    private SwerveModule.DriveRequestType driveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    private SwerveModule.SteerRequestType steerRequestType = SwerveModule.SteerRequestType.Position;
    /**
     * Whether to desaturate wheel speeds before applying.
     * For more information, see the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    private boolean desaturateWheelSpeeds = true;

    /**
     * The perspective to use when determining which direction is forward for driving.
     * Unlike {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#ForwardPerspective FieldCentricFacingAngle}, this does NOT influence target direction.
     */
    private ForwardPerspectiveValue drivingPerspective = ForwardPerspectiveValue.OperatorPerspective;

    private final FieldCentric fieldCentric = new FieldCentric().withRotationalDeadband(0);

    private final PhoenixPIDController headingController;
    private final double maxAngularVelocity;

    private boolean resetRequested = false;
    private boolean motionIsFinished = false;

    // Optional NetworkTables logging
    private final DoublePublisher goalPositionPub;
    private final DoublePublisher appliedVelocityPub;
    private final BooleanPublisher motionIsFinishedPub;

    /**
     * Creates a new request with the given gains.
     *
     * @param kp The P gain for the heading controller in radians per second output per radian error.
     * @param ki The I gain for the heading controller in radians per second output per integral of radian error.
     * @param kp The P gain for the heading controller in radians per second output per the derivative of error radians per second.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     */
    public DriveFacingPosition(double kp, double ki, double kd, double maxAngularVelocity) {
        headingController = new PhoenixPIDController(kp, ki, kd);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.maxAngularVelocity = maxAngularVelocity;

        goalPositionPub = null;
        appliedVelocityPub = null;
        motionIsFinishedPub = null;
    }

    /**
     * Creates a new profiled request with the given gains,
     * and logs motion profile data in a subtable named "Facing Position".
     *
     * @param kp The P gain for the heading controller in radians per second output per radian error.
     * @param ki The I gain for the heading controller in radians per second output per integral of radian error.
     * @param kp The P gain for the heading controller in radians per second output per the derivative of error radians per second.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     */
    public DriveFacingPosition(double kp, double ki, double kd, double maxAngularVelocity, NetworkTable loggingPath) {
        headingController = new PhoenixPIDController(kp, ki, kd);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.maxAngularVelocity = maxAngularVelocity;

        NetworkTable motionTable = loggingPath.getSubTable("Facing Position");
        this.appliedVelocityPub =
                motionTable.getDoubleTopic("Applied Velocity (rads per sec)").publish();

        this.motionIsFinishedPub =
                motionTable.getBooleanTopic("Motion is Finished").publish();

        NetworkTable goalTable = motionTable.getSubTable("Goal");
        this.goalPositionPub = goalTable.getDoubleTopic("Position (radians)").publish();
    }

    /**
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        // Find the angle of the vector that the goal would make if the robot was the origin
        double xDistance = targetPosition.getX() - parameters.currentPose.getX();
        double yDistance = targetPosition.getY() - parameters.currentPose.getY();
        double yawRadians = Math.atan2(yDistance, xDistance);
        Rotation2d targetDirection = Rotation2d.fromRadians(yawRadians);

        Rotation2d currentAngle = parameters.currentPose.getRotation();

        if (resetRequested) {
            headingController.reset();
            this.resetRequested = false;
        }

        double toApplyOmega = headingController.calculate(
                currentAngle.getRadians(), targetDirection.getRadians(), parameters.timestamp);

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

        // If one of the publishers isn't null, all of them were initialized, so log data
        if (this.goalPositionPub != null) {
            long timestamp = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);

            goalPositionPub.set(targetDirection.getRadians(), timestamp);
            appliedVelocityPub.set(toApplyOmega, timestamp);
            motionIsFinishedPub.set(motionIsFinished, timestamp);
        }

        return fieldCentric
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(toApplyOmega)
                .withDeadband(deadband)
                .withCenterOfRotation(centerOfRotation)
                .withDriveRequestType(driveRequestType)
                .withSteerRequestType(steerRequestType)
                .withDesaturateWheelSpeeds(desaturateWheelSpeeds)
                .withForwardPerspective(drivingPerspective)
                .apply(parameters, modulesToApply);
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
    public DriveFacingPosition withVelocityX(double newVelocityX) {
        this.velocityX = newVelocityX;
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
        this.velocityX = newVelocityX.in(MetersPerSecond);
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
        this.velocityY = newVelocityY;
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
        this.velocityY = newVelocityY.in(MetersPerSecond);
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
        this.targetPosition = newTargetPosition;
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
        this.deadband = newDeadband;
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
        this.deadband = newDeadband.in(MetersPerSecond);
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
        this.centerOfRotation = newCenterOfRotation;
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
        this.driveRequestType = newDriveRequestType;
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
        this.steerRequestType = newSteerRequestType;
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
        this.desaturateWheelSpeeds = newDesaturateWheelSpeeds;
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
        this.drivingPerspective = newDrivingPerspective;
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
        this.headingController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public DriveFacingPosition withTolerance(Angle toleranceAmount) {
        this.headingController.setTolerance(toleranceAmount.in(Radians));
        return this;
    }

    /**
     * @return Whether or not the robot has reached its target rotation,
     * based on the tolerance set using {@link frc.robot.subsystems.drive.requests.DriveFacingPosition#withTolerance(Angle) withTolerance()}
     */
    public boolean motionIsFinished() {
        return this.motionIsFinished;
    }
}
