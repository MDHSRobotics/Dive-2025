package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.DriveTelemetry;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Drives the swerve drivetrain in a field-centric manner,
 * maintaining a heading angle to ensure that limelight tx is 0.
 * Rotation to the target direction is profiled using a trapezoid profile.
 * <p>
 * An example scenario is that the robot sees an apriltag at tx = 10 (degrees clockwise).
 * The robot would then rotate 10 degrees clockwise to face the tag.
 * <p>
 * If no tag is visible, the robot will finish rotating based on the last known tx value, and then stop rotating.
 * <p>
 * This swerve request is based on {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle},
 * and makes some other improvements besides the motion profile.
 * <p>
 * We opted to take some code from {@link edu.wpi.first.math.controller.ProfiledPIDController ProfiledPIDController}
 * so we could use a {@link PhoenixPIDController PhoenixPIDController} instead of a normal PID controller.
 * <p>
 * Note: this request makes use of a
 * <a href="https://docs.wpilib.org/en/stable/docs/software/networktables/listening-for-change.html#using-networktableinstance-to-listen-for-changes">NetworkTable listener</a>
 * so that it can differentiate between new and old tx values,
 * even when the values are the same.
 * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-with-visual-servoing">Explanation of visual servoing</a>
 * @see <a href="https://www.chiefdelphi.com/t/implementing-feedforward-with-ctre-s-fieldcentricfacingangle-request/475822/14">Original source of this code</a>
 */
public class ProfiledDriveWithVisualServoing implements ProfiledSwerveRequest {
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
     * The target direction for the swerve request.
     * This field is not modifiable outside of this class because it is determined by tx.
     */
    private Rotation2d targetDirection = new Rotation2d();
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

    private final PhoenixPIDController headingController = new PhoenixPIDController(0, 0, 0);

    /* Profile used for the target direction */
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private final TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private final double kDt;

    private boolean resetRequested = false;
    private boolean motionIsFinished = false;

    private final AtomicReference<Double> txValue = new AtomicReference<Double>(0.0);

    // Optional NetworkTables logging
    private final DoublePublisher goalPositionPub;
    private final DoublePublisher setpointPositionPub;
    private final DoublePublisher setpointVelocityPub;
    private final DoublePublisher errorCorrectionVelocityPub;
    private final DoublePublisher appliedVelocityPub;

    /**
     * Creates a new profiled request with the given constraints and camera.
     *
     * @param constraints Constraints for the trapezoid profile
     * @param kDt Update period for the motion profile
     * @param cameraTable The NetworkTable for the limelight.
     */
    public ProfiledDriveWithVisualServoing(
            TrapezoidProfile.Constraints constraints, double kDt, NetworkTable cameraTable) {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        profile = new TrapezoidProfile(constraints);
        this.kDt = kDt;

        DoubleSubscriber txSub = cameraTable.getDoubleTopic("tx").subscribe(0);

        cameraTable
                .getInstance()
                .addListener(
                        txSub,
                        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                        event -> txValue.set(event.valueData.value.getDouble()));

        goalPositionPub = null;
        setpointPositionPub = null;
        setpointVelocityPub = null;
        errorCorrectionVelocityPub = null;
        appliedVelocityPub = null;
    }

    /**
     * Creates a new profiled request with the given constraints and camera,
     * and logs motion profile data in a subtable named "Facing Angle".
     *
     * @param constraints Constraints for the trapezoid profile
     * @param kDt Update period for the motion profile
     * @param cameraTable The NetworkTable for the limelight.
     * @param loggingPath The NetworkTable to log data into.
     */
    public ProfiledDriveWithVisualServoing(
            TrapezoidProfile.Constraints constraints, double kDt, NetworkTable cameraTable, NetworkTable loggingPath) {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        profile = new TrapezoidProfile(constraints);
        this.kDt = kDt;

        DoubleSubscriber txSub = cameraTable.getDoubleTopic("tx").subscribe(0);

        cameraTable
                .getInstance()
                .addListener(
                        txSub,
                        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                        event -> txValue.set(event.valueData.value.getDouble()));

        NetworkTable motionTable = loggingPath.getSubTable("Visual Servoing");
        NetworkTable goalTable = motionTable.getSubTable("Goal");
        this.goalPositionPub = goalTable.getDoubleTopic("Position (radians)").publish();
        NetworkTable setpointTable = motionTable.getSubTable("Setpoint");
        this.setpointPositionPub =
                setpointTable.getDoubleTopic("Position (radians)").publish();
        this.setpointVelocityPub =
                setpointTable.getDoubleTopic("Velocity (rads per sec)").publish();
        this.errorCorrectionVelocityPub = motionTable
                .getDoubleTopic("Error Correction Velocity (rads per sec)")
                .publish();
        this.appliedVelocityPub =
                motionTable.getDoubleTopic("Applied Velocity (rads per sec)").publish();
    }

    /**
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double, double)
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double)
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Rotation2d currentAngle = parameters.currentPose.getRotation();
        double currentAngularVelocity = parameters.currentChassisSpeed.omegaRadiansPerSecond;
        Double tx = txValue.getAndSet(null);
        // If tx has updated, update the target direction.
        if (tx != null) {
            // You need to subtract instead of adding because the current angle is counterclockwise, but tx is
            // clockwise.
            this.targetDirection = currentAngle.minus(Rotation2d.fromDegrees(tx));
        }

        if (resetRequested) {
            setpoint.position = currentAngle.getRadians();
            setpoint.velocity = currentAngularVelocity;
            this.resetRequested = false;
        }

        /* From ProfiledPIDController#calculate(double)
         * The following code handles wrapping values (like angles) by eliminating unnecessary rotation.
         * Basically, if the end goal is more than 180 degrees from the current angle, it moves the goal closer.
         * The same happens with setpoint.
         */
        // Get the smallest possible distance between goal and measurement
        double goalMinDistance = MathUtil.angleModulus(this.targetDirection.getRadians() - currentAngle.getRadians());
        double setpointMinDistance = MathUtil.angleModulus(setpoint.position - currentAngle.getRadians());

        // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
        // may be outside the input range after this operation, but that's OK because the controller
        // will still go there and report an error of zero. In other words, the setpoint only needs to
        // be offset from the measurement by the input range modulus; they don't need to be equal.
        goal.position = goalMinDistance + currentAngle.getRadians();
        setpoint.position = setpointMinDistance + currentAngle.getRadians();

        // Progress the setpoint of the motion profile
        setpoint = profile.calculate(kDt, setpoint, goal);

        // Calculate the extra angular velocity necessary to get the robot to the correct angle.
        double errorCorrectionOutput =
                headingController.calculate(currentAngle.getRadians(), setpoint.position, parameters.timestamp);

        double toApplyOmega = setpoint.velocity + errorCorrectionOutput;

        this.motionIsFinished = (headingController.atSetpoint() && goal.equals(setpoint));

        // If one of the publishers isn't null, all of them were initialized, so log data
        if (this.goalPositionPub != null) {
            long timestamp = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);

            goalPositionPub.set(goal.position, timestamp);
            setpointPositionPub.set(setpoint.position, timestamp);
            setpointVelocityPub.set(setpoint.velocity, timestamp);
            errorCorrectionVelocityPub.set(errorCorrectionOutput, timestamp);
            appliedVelocityPub.set(toApplyOmega, timestamp);
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
    public ProfiledDriveWithVisualServoing withVelocityX(double newVelocityX) {
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
    public ProfiledDriveWithVisualServoing withVelocityX(LinearVelocity newVelocityX) {
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
    public ProfiledDriveWithVisualServoing withVelocityY(double newVelocityY) {
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
    public ProfiledDriveWithVisualServoing withVelocityY(LinearVelocity newVelocityY) {
        this.velocityY = newVelocityY.in(MetersPerSecond);
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
    public ProfiledDriveWithVisualServoing withDeadband(double newDeadband) {
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
    public ProfiledDriveWithVisualServoing withDeadband(LinearVelocity newDeadband) {
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
    public ProfiledDriveWithVisualServoing withCenterOfRotation(Translation2d newCenterOfRotation) {
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
    public ProfiledDriveWithVisualServoing withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
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
    public ProfiledDriveWithVisualServoing withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
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
    public ProfiledDriveWithVisualServoing withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
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
    public ProfiledDriveWithVisualServoing withDrivingPerspective(ForwardPerspectiveValue newDrivingPerspective) {
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
    public ProfiledDriveWithVisualServoing withPIDGains(double kp, double ki, double kd) {
        this.headingController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public ProfiledDriveWithVisualServoing withTolerance(Angle toleranceAmount) {
        this.headingController.setTolerance(toleranceAmount.in(Radians));
        return this;
    }

    /**
     * @return Whether or not the robot has reached its target rotation,
     * based on the tolerance set using {@link frc.robot.subsystems.drive.requests.ProfiledDriveFacingAngle#withTolerance(Angle) withTolerance()}
     */
    public boolean motionIsFinished() {
        return this.motionIsFinished;
    }
}
