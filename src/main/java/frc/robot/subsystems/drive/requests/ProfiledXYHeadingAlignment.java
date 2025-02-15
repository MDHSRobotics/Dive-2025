package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveTelemetry;

/**
 * Drives the swerve drivetrain in a field-centric manner, maintaining a
 * specified x and y position and heading angle to ensure the robot is in the right position and facing the desired direction.
 * Translational and rotaional alignment is profiled using separate trapezoid profiles.
 * <p>
 * This swerve request is based on {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle},
 * and makes some other improvements besides the motion profiles.
 * <p>
 * We opted to take some code from {@link edu.wpi.first.math.controller.ProfiledPIDController ProfiledPIDController}
 * so we could use a {@link PhoenixPIDController PhoenixPIDController} instead of a normal PID controller.
 * @see <a href="https://www.chiefdelphi.com/t/implementing-feedforward-with-ctre-s-fieldcentricfacingangle-request/475822/14">Original source of this code</a>
 */
public class ProfiledXYHeadingAlignment implements ProfiledSwerveRequest {
    /**
     * The desired pose to reach.
     * This pose has the blue alliance origin.
     */
    private Pose2d targetPose = new Pose2d();

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

    private final FieldCentric fieldCentric = new FieldCentric();

    // X position profile and PID controller
    private final TrapezoidProfile xProfile;
    private TrapezoidProfile.State xSetpoint = new TrapezoidProfile.State();
    private final TrapezoidProfile.State xGoal = new TrapezoidProfile.State();
    private final PhoenixPIDController xController = new PhoenixPIDController(0, 0, 0);

    // Y position profile and PID controller
    private final TrapezoidProfile yProfile;
    private TrapezoidProfile.State ySetpoint = new TrapezoidProfile.State();
    private final TrapezoidProfile.State yGoal = new TrapezoidProfile.State();
    private final PhoenixPIDController yController = new PhoenixPIDController(0, 0, 0);

    // Rotation profile and PID controller
    private final TrapezoidProfile headingProfile;
    private TrapezoidProfile.State headingSetpoint = new TrapezoidProfile.State();
    private final TrapezoidProfile.State headingGoal = new TrapezoidProfile.State();
    private final PhoenixPIDController headingController = new PhoenixPIDController(0, 0, 0);

    private final double kDt;

    private boolean resetRequested = false;

    // Optional NetworkTables logging
    private final StructPublisher<Pose2d> goalPositionPub;
    private final StructPublisher<Pose2d> setpointPositionPub;
    private final StructPublisher<ChassisSpeeds> setpointVelocityPub;
    private final StructPublisher<ChassisSpeeds> errorCorrectionVelocityPub;
    private final StructPublisher<ChassisSpeeds> appliedVelocityPub;

    /**
     * Creates a new profiled request with the given constraints.
     *
     * @param linearConstraints Constraints for the X and Y trapezoid profiles
     * @param angularConstraints Constraints for the heading profile
     * @param kDt Update period for the motion profile
     */
    public ProfiledXYHeadingAlignment(
            TrapezoidProfile.Constraints linearConstraints,
            TrapezoidProfile.Constraints angularConstraints,
            double kDt) {
        xProfile = new TrapezoidProfile(linearConstraints);
        yProfile = new TrapezoidProfile(linearConstraints);
        headingProfile = new TrapezoidProfile(angularConstraints);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.kDt = kDt;

        // Make PID gains tunable from NetworkTables
        SmartDashboard.putData("X Controller", xController);
        SmartDashboard.putData("Y Controller", yController);
        SmartDashboard.putData("Heading Controller", headingController);

        goalPositionPub = null;
        setpointPositionPub = null;
        setpointVelocityPub = null;
        errorCorrectionVelocityPub = null;
        appliedVelocityPub = null;
    }

    /**
     * Creates a new profiled request with the given constraints.
     *
     * @param linearConstraints Constraints for the X and Y trapezoid profiles
     * @param angularConstraints Constraints for the heading profile
     * @param kDt Update period for the motion profile
     * @param loggingPath The NetworkTable to log data into.
     */
    public ProfiledXYHeadingAlignment(
            TrapezoidProfile.Constraints linearConstraints,
            TrapezoidProfile.Constraints angularConstraints,
            double kDt,
            NetworkTable loggingPath) {
        xProfile = new TrapezoidProfile(linearConstraints);
        yProfile = new TrapezoidProfile(linearConstraints);
        headingProfile = new TrapezoidProfile(angularConstraints);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.kDt = kDt;

        // Make PID gains tunable from NetworkTables
        SmartDashboard.putData("X Controller", xController);
        SmartDashboard.putData("Y Controller", yController);
        SmartDashboard.putData("Heading Controller", headingController);

        NetworkTable motionTable = loggingPath.getSubTable("Facing Angle");
        NetworkTable goalTable = motionTable.getSubTable("Goal");
        this.goalPositionPub =
                goalTable.getStructTopic("Position", Pose2d.struct).publish();
        NetworkTable setpointTable = motionTable.getSubTable("Setpoint");
        this.setpointPositionPub =
                setpointTable.getStructTopic("Position", Pose2d.struct).publish();
        this.setpointVelocityPub =
                setpointTable.getStructTopic("Velocity", ChassisSpeeds.struct).publish();
        this.errorCorrectionVelocityPub = motionTable
                .getStructTopic("Error Correction Velocity", ChassisSpeeds.struct)
                .publish();
        this.appliedVelocityPub = motionTable
                .getStructTopic("Applied Velocity", ChassisSpeeds.struct)
                .publish();
    }

    /**
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double, double)
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double)
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Pose2d currentPose = parameters.currentPose;
        Rotation2d currentAngle = parameters.currentPose.getRotation();
        double currentAngularVelocity = parameters.currentChassisSpeed.omegaRadiansPerSecond;

        if (resetRequested) {
            xSetpoint.position = currentPose.getX();
            xSetpoint.velocity = parameters.currentChassisSpeed.vxMetersPerSecond;
            ySetpoint.position = currentPose.getY();
            ySetpoint.velocity = parameters.currentChassisSpeed.vyMetersPerSecond;
            headingSetpoint.position = currentAngle.getRadians();
            headingSetpoint.velocity = currentAngularVelocity;
            this.resetRequested = false;
        }

        xSetpoint = xProfile.calculate(kDt, xSetpoint, xGoal);
        double xCorrectionOutput = xController.calculate(currentPose.getX(), xSetpoint.position, parameters.timestamp);
        double toApplyX = xSetpoint.velocity + xCorrectionOutput;

        ySetpoint = yProfile.calculate(kDt, ySetpoint, yGoal);
        double yCorrectionOutput = yController.calculate(currentPose.getY(), ySetpoint.position, parameters.timestamp);
        double toApplyY = ySetpoint.velocity + yCorrectionOutput;

        /* From ProfiledPIDController#calculate(double)
         * The following code handles wrapping values (like angles) by eliminating unnecessary rotation.
         * Basically, if the end goal is more than 180 degrees from the current angle, it moves the goal closer.
         * The same happens with setpoint.
         */
        // Get the smallest possible distance between goal and measurement
        double goalMinDistance =
                MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentAngle.getRadians());
        double setpointMinDistance = MathUtil.angleModulus(headingSetpoint.position - currentAngle.getRadians());

        // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
        // may be outside the input range after this operation, but that's OK because the controller
        // will still go there and report an error of zero. In other words, the setpoint only needs to
        // be offset from the measurement by the input range modulus; they don't need to be equal.
        headingGoal.position = goalMinDistance + currentAngle.getRadians();
        headingSetpoint.position = setpointMinDistance + currentAngle.getRadians();

        // Progress the setpoint of the motion profile
        headingSetpoint = headingProfile.calculate(kDt, headingSetpoint, headingGoal);

        // Calculate the extra angular velocity necessary to get the robot to the correct angle.
        double headingCorrectionOutput =
                headingController.calculate(currentAngle.getRadians(), headingSetpoint.position, parameters.timestamp);

        double toApplyOmega = headingSetpoint.velocity + headingCorrectionOutput;

        // If one of the publishers isn't null, all of them were initialized, so log data
        if (this.goalPositionPub != null) {
            long timestamp = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);

            goalPositionPub.set(
                    new Pose2d(xGoal.position, yGoal.position, Rotation2d.fromRadians(headingGoal.position)),
                    timestamp);
            setpointPositionPub.set(
                    new Pose2d(
                            xSetpoint.position, ySetpoint.position, Rotation2d.fromRadians(headingSetpoint.position)),
                    timestamp);
            setpointVelocityPub.set(
                    new ChassisSpeeds(xSetpoint.velocity, ySetpoint.velocity, headingSetpoint.velocity), timestamp);
            errorCorrectionVelocityPub.set(
                    new ChassisSpeeds(xCorrectionOutput, yCorrectionOutput, headingCorrectionOutput), timestamp);
            appliedVelocityPub.set(new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega), timestamp);
        }

        return fieldCentric
                .withVelocityX(toApplyX)
                .withVelocityY(toApplyY)
                .withRotationalRate(toApplyOmega)
                .withDeadband(0)
                .withRotationalDeadband(0)
                .withCenterOfRotation(centerOfRotation)
                .withDriveRequestType(driveRequestType)
                .withSteerRequestType(steerRequestType)
                .withDesaturateWheelSpeeds(desaturateWheelSpeeds)
                .withForwardPerspective(
                        ForwardPerspectiveValue
                                .BlueAlliance) // Must be blue alliance so that CTRE doesn't flip our Y velocity.
                .apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset the profile used for the target direction next time it is used.
     */
    public void resetProfile() {
        this.resetRequested = true;
    }

    /**
     * Modifies the TargetPose parameter and returns itself.
     * <p>
     * The desired direction to face. 0 Degrees is defined as in the direction of
     * the X axis. As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     *
     * @param newTargetPose Parameter to modify
     * @return this object
     */
    public ProfiledXYHeadingAlignment withTargetPose(Pose2d newTargetPose) {
        this.targetPose = newTargetPose;
        this.xGoal.position = newTargetPose.getX();
        this.yGoal.position = newTargetPose.getY();
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
    public ProfiledXYHeadingAlignment withCenterOfRotation(Translation2d newCenterOfRotation) {
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
    public ProfiledXYHeadingAlignment withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
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
    public ProfiledXYHeadingAlignment withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
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
    public ProfiledXYHeadingAlignment withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.desaturateWheelSpeeds = newDesaturateWheelSpeeds;
        return this;
    }

    /**
     * Modifies the PID gains for the y controller and returns itself.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @return this object
     */
    public ProfiledXYHeadingAlignment withTranslationalPIDGains(double kp, double ki, double kd) {
        this.xController.setPID(kp, ki, kd);
        this.yController.setPID(kp, ki, kd);
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
    public ProfiledXYHeadingAlignment withRotationalPIDGains(double kp, double ki, double kd) {
        this.headingController.setPID(kp, ki, kd);
        return this;
    }
}
