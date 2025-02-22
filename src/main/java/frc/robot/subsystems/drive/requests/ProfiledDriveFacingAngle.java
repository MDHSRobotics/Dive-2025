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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.DriveTelemetry;

/**
 * Drives the swerve drivetrain in a field-centric manner, maintaining a
 * specified heading angle to ensure the robot is facing the desired direction.
 * Rotation to the target direction is profiled using a trapezoid profile.
 * <p>
 * An example scenario is that the robot is oriented to the east, the VelocityX
 * is +5 m/s, VelocityY is 0 m/s, and TargetDirection is 180 degrees.
 * In this scenario, the robot would drive northward at 5 m/s and turn clockwise
 * to a target of 180 degrees.
 * <p>
 * This swerve request is based on {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle},
 * and makes some other improvements besides the motion profile.
 * <p>
 * We opted to take some code from {@link edu.wpi.first.math.controller.ProfiledPIDController ProfiledPIDController}
 * so we could use a {@link PhoenixPIDController PhoenixPIDController} instead of a normal PID controller.
 * @see <a href="https://www.chiefdelphi.com/t/implementing-feedforward-with-ctre-s-fieldcentricfacingangle-request/475822/14">Original source of this code</a>
 */
public class ProfiledDriveFacingAngle implements ProfiledSwerveRequest {
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
     * The desired direction to face.
     * 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
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

    /**
     * The perspective to use when determing which direction is forward for aiming.
     * By default, this swerve request considers the angle to be in <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">the blue coordinate system.</a>
     */
    private ForwardPerspectiveValue aimingPerspective = ForwardPerspectiveValue.BlueAlliance;

    private final FieldCentric fieldCentric = new FieldCentric();

    private final PhoenixPIDController headingController = new PhoenixPIDController(0, 0, 0);
    private final DoubleEntry pGainEntry;

    /* Profile used for the target direction */
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private final TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private final double kDt;

    private boolean resetRequested = false;
    private boolean motionIsFinished = false;

    // Optional NetworkTables logging
    private final DoublePublisher goalPositionPub;
    private final DoublePublisher setpointPositionPub;
    private final DoublePublisher setpointVelocityPub;
    private final DoublePublisher errorCorrectionVelocityPub;
    private final DoublePublisher appliedVelocityPub;
    private final BooleanPublisher motionIsFinishedPub;

    /**
     * Creates a new profiled request with the given constraints.
     *
     * @param constraints Constraints for the trapezoid profile
     * @param kDt Update period for the motion profile
     * @param goalTolerance What angle is acceptable to stop rotating
     */
    public ProfiledDriveFacingAngle(TrapezoidProfile.Constraints constraints, double kDt, Angle goalTolerance) {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(goalTolerance.in(Radians));
        profile = new TrapezoidProfile(constraints);
        this.kDt = kDt;

        pGainEntry = null;
        goalPositionPub = null;
        setpointPositionPub = null;
        setpointVelocityPub = null;
        errorCorrectionVelocityPub = null;
        appliedVelocityPub = null;
        motionIsFinishedPub = null;
    }

    /**
     * Creates a new profiled request with the given constraints,
     * and logs motion profile data in a subtable named "Facing Angle".
     *
     * @param constraints Constraints for the trapezoid profile
     * @param kDt Update period for the motion profile
     * @param loggingPath The NetworkTable to log data into.
     * @param goalTolerance What angle is acceptable to stop rotating
     */
    public ProfiledDriveFacingAngle(
            TrapezoidProfile.Constraints constraints, double kDt, NetworkTable loggingPath, Angle goalTolerance) {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(goalTolerance.in(Radians));
        profile = new TrapezoidProfile(constraints);
        this.kDt = kDt;

        NetworkTable motionTable = loggingPath.getSubTable("Facing Angle");

        NetworkTable pidTable = motionTable.getSubTable("PID Gains");
        this.pGainEntry =
                pidTable.getDoubleTopic("P").getEntry(this.headingController.getP(), PubSubOption.excludeSelf(true));
        this.pGainEntry.set(this.headingController.getP());

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

        this.motionIsFinishedPub =
                motionTable.getBooleanTopic("Motion is Finished").publish();
    }

    /**
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double, double)
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double)
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Rotation2d currentAngle = parameters.currentPose.getRotation();
        double currentAngularVelocity = parameters.currentChassisSpeed.omegaRadiansPerSecond;
        Rotation2d fakeTargetDirection = targetDirection;

        if (resetRequested) {
            setpoint.position = currentAngle.getRadians();
            setpoint.velocity = currentAngularVelocity;
            this.resetRequested = false;
        }

        /* If the user requested a target direction according to the operator perspective, rotate our target direction by the angle */
        if (aimingPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            fakeTargetDirection = targetDirection.rotateBy(parameters.operatorForwardDirection);
        }

        /* From ProfiledPIDController#calculate(double)
         * The following code handles wrapping values (like angles) by eliminating unnecessary rotation.
         * Basically, if the end goal is more than 180 degrees from the current angle, it moves the goal closer.
         * The same happens with setpoint.
         */
        // Get the smallest possible distance between goal and measurement
        double goalMinDistance = MathUtil.angleModulus(fakeTargetDirection.getRadians() - currentAngle.getRadians());
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
        if (pGainEntry != null) {
            headingController.setPID(pGainEntry.get(), 0, 0);
        }
        double errorCorrectionOutput =
                headingController.calculate(currentAngle.getRadians(), setpoint.position, parameters.timestamp);
        if (headingController.atSetpoint()) {
            errorCorrectionOutput = 0;
        }

        double toApplyOmega = setpoint.velocity + errorCorrectionOutput;

        this.motionIsFinished = headingController.atSetpoint() && goal.equals(setpoint);

        // If one of the publishers isn't null, all of them were initialized, so log data
        if (this.goalPositionPub != null) {
            long timestamp = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);

            goalPositionPub.set(goal.position, timestamp);
            setpointPositionPub.set(setpoint.position, timestamp);
            setpointVelocityPub.set(setpoint.velocity, timestamp);
            errorCorrectionVelocityPub.set(errorCorrectionOutput, timestamp);
            appliedVelocityPub.set(toApplyOmega, timestamp);
            motionIsFinishedPub.set(motionIsFinished, timestamp);
        }

        return fieldCentric
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(toApplyOmega)
                .withDeadband(deadband)
                .withRotationalDeadband(0)
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
    public ProfiledDriveFacingAngle withVelocityX(double newVelocityX) {
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
    public ProfiledDriveFacingAngle withVelocityX(LinearVelocity newVelocityX) {
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
    public ProfiledDriveFacingAngle withVelocityY(double newVelocityY) {
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
    public ProfiledDriveFacingAngle withVelocityY(LinearVelocity newVelocityY) {
        this.velocityY = newVelocityY.in(MetersPerSecond);
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
    public ProfiledDriveFacingAngle withTargetDirection(Rotation2d newTargetDirection) {
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
    public ProfiledDriveFacingAngle withDeadband(double newDeadband) {
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
    public ProfiledDriveFacingAngle withDeadband(LinearVelocity newDeadband) {
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
    public ProfiledDriveFacingAngle withCenterOfRotation(Translation2d newCenterOfRotation) {
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
    public ProfiledDriveFacingAngle withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
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
    public ProfiledDriveFacingAngle withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
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
    public ProfiledDriveFacingAngle withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
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
    public ProfiledDriveFacingAngle withDrivingPerspective(ForwardPerspectiveValue newDrivingPerspective) {
        this.drivingPerspective = newDrivingPerspective;
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
    public ProfiledDriveFacingAngle withAimingPerspective(ForwardPerspectiveValue newAimingPerspective) {
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
    public ProfiledDriveFacingAngle withPIDGains(double kp, double ki, double kd) {
        this.headingController.setPID(kp, ki, kd);
        if (pGainEntry != null) {
            pGainEntry.set(kp);
        }
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public ProfiledDriveFacingAngle withTolerance(Angle toleranceAmount) {
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
