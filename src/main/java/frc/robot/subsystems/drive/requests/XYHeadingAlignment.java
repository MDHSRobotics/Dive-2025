package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.DriveTelemetry;

/**
 * Drives the swerve drivetrain in a field-centric manner, maintaining a
 * specified x and y position and heading angle to ensure the robot is in the right position and facing the desired direction.
 * <p>
 * This swerve request uses {@link edu.wpi.first.math.trajectory.TrapezoidProfile trapezoid profiles} for x and y,
 * a {@link com.ctre.phoenix6.swerve.utility.PhoenixPIDController PhoenixPIDController} for heading,
 * and PathPlanner's {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator SwerveSetpointGenerator} to limit all three velocities.
 * <p>
 * The request is based on {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle},
 * and makes some other improvements besides the motion profiles.
 * <p>
 * Important: You may not change the targetPose while the swerve request is in use.
 * <p>
 * This request currently supports drivetrains with 4 modules.
 */
public class XYHeadingAlignment implements ResettableSwerveRequest {
    /**
     * The robot-relative chassis speeds to apply to the drivetrain.
     */
    private ChassisSpeeds toApplyRobotSpeeds = new ChassisSpeeds();
    /**
     * The field-relative chassis speeds to log to NetworkTables.
     */
    private ChassisSpeeds toApplyFieldSpeeds = new ChassisSpeeds();
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
    private boolean desaturateWheelSpeeds = false;

    /** Must be robot-relative because the swerve setpoint generator outputs robot-relative wheel forces */
    private final ApplyRobotSpeeds applyRobotSpeeds = new ApplyRobotSpeeds();

    // X position profile and PID controller
    private final TrapezoidProfile xProfile;
    private final TrapezoidProfile.State xStartingState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State xGoal = new TrapezoidProfile.State();
    private final PhoenixPIDController xController;

    // Y position profile and PID controller
    private final TrapezoidProfile yProfile;
    private final TrapezoidProfile.State yStartingState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State yGoal = new TrapezoidProfile.State();
    private final PhoenixPIDController yController;

    // Rotation PID controller
    private final PhoenixPIDController headingController;
    private final double maxAngularVelocity;

    /**
     * The timestamp for the start of this request, in the timebase of {@link Utils#getCurrentTimeSeconds()}.
     * This is used for the trapezoid profiles.
     */
    private double startingTimestamp = 0;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveModuleState[] startingModuleStates = new SwerveModuleState[4];
    private SwerveSetpoint previousSwerveSetpoint;
    /**
     * The update period for the {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator swerve setpoint generator} in seconds.
     */
    private final double updatePeriod;

    private boolean resetRequested = false;

    // NetworkTables logging
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Swerve Requests").getSubTable("X Y Heading Alignment");
    private final StructPublisher<Pose2d> goalPositionPub =
            table.getSubTable("Goal").getStructTopic("Position", Pose2d.struct).publish();

    private final NetworkTable setpointTable = table.getSubTable("Setpoint");
    private final StructPublisher<Pose2d> setpointPositionPub =
            setpointTable.getStructTopic("Position", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> setpointVelocityPub =
            setpointTable.getStructTopic("Velocity", ChassisSpeeds.struct).publish();

    private final StructPublisher<ChassisSpeeds> errorCorrectionVelocityPub = table.getStructTopic(
                    "Error Correction Velocity", ChassisSpeeds.struct)
            .publish();
    private final StructPublisher<ChassisSpeeds> appliedVelocityPub =
            table.getStructTopic("Applied Velocity", ChassisSpeeds.struct).publish();

    /**
     * Creates a new profiled request with the given constraints.
     *
     * @param kTranslationP The P gain for the translation controller in meters per second output per meter error.
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     * @param linearConstraints Constraints for the X and Y trapezoid profiles
     * @param robotConfig The PathPlanner config for the robot
     * @param maxSteerVelocityRadsPerSec The maximum rotation velocity of a swerve module, in radians per second
     * @param updatePeriod The amount of time between robot updates in seconds.
     */
    public XYHeadingAlignment(
            double kTranslationP,
            double kRotationP,
            double maxAngularVelocity,
            TrapezoidProfile.Constraints linearConstraints,
            RobotConfig robotConfig,
            double maxSteerVelocityRadsPerSec,
            double updatePeriod) {
        xProfile = new TrapezoidProfile(linearConstraints);
        xController = new PhoenixPIDController(kTranslationP, 0.0, 0.0);
        yProfile = new TrapezoidProfile(linearConstraints);
        yController = new PhoenixPIDController(kTranslationP, 0.0, 0.0);

        headingController = new PhoenixPIDController(kRotationP, 0.0, 0.0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.maxAngularVelocity = maxAngularVelocity;

        setpointGenerator = new SwerveSetpointGenerator(robotConfig, maxSteerVelocityRadsPerSec);
        this.updatePeriod = updatePeriod;
    }

    /**
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double, double)
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double)
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Pose2d currentPose = parameters.currentPose;
        Rotation2d currentAngle = parameters.currentPose.getRotation();
        Rotation2d targetDirection = targetPose.getRotation();

        if (resetRequested) {
            xStartingState.position = currentPose.getX();
            xStartingState.velocity = parameters.currentChassisSpeed.vxMetersPerSecond;
            yStartingState.position = currentPose.getY();
            yStartingState.velocity = parameters.currentChassisSpeed.vyMetersPerSecond;
            startingTimestamp = parameters.timestamp;

            for (int i = 0; i < 4; ++i) {
                startingModuleStates[i] = modulesToApply[i].getCurrentState();
            }
            previousSwerveSetpoint = new SwerveSetpoint(
                    parameters.currentChassisSpeed, startingModuleStates, DriveFeedforwards.zeros(4));

            xController.reset();
            yController.reset();
            headingController.reset();
            this.resetRequested = false;
        }

        double time = parameters.timestamp - startingTimestamp;
        TrapezoidProfile.State xSetpoint = xProfile.calculate(time, xStartingState, xGoal);
        double xCorrectionOutput = xController.calculate(currentPose.getX(), xSetpoint.position, parameters.timestamp);
        if (xController.atSetpoint()) {
            xCorrectionOutput = 0;
        }
        toApplyFieldSpeeds.vxMetersPerSecond = xSetpoint.velocity + xCorrectionOutput;

        TrapezoidProfile.State ySetpoint = yProfile.calculate(time, yStartingState, yGoal);
        double yCorrectionOutput = yController.calculate(currentPose.getY(), ySetpoint.position, parameters.timestamp);
        if (yController.atSetpoint()) {
            yCorrectionOutput = 0;
        }
        toApplyFieldSpeeds.vyMetersPerSecond = ySetpoint.velocity + yCorrectionOutput;

        // Calculate the extra angular velocity necessary to get the robot to the correct angle.
        double headingCorrectionOutput = headingController.calculate(
                currentAngle.getRadians(), targetDirection.getRadians(), parameters.timestamp);

        if (headingController.atSetpoint()) {
            headingCorrectionOutput = 0;
        }

        toApplyFieldSpeeds.omegaRadiansPerSecond =
                MathUtil.clamp(headingCorrectionOutput, -maxAngularVelocity, maxAngularVelocity);

        // The generator requires robot-relative speeds
        toApplyRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(toApplyFieldSpeeds, currentAngle);

        previousSwerveSetpoint =
                setpointGenerator.generateSetpoint(previousSwerveSetpoint, toApplyRobotSpeeds, updatePeriod);

        toApplyRobotSpeeds = previousSwerveSetpoint.robotRelativeSpeeds();

        // Convert back to field-relative speeds for the sake of easier logging.
        toApplyFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(toApplyRobotSpeeds, currentAngle);

        // If one of the publishers isn't null, all of them were initialized, so log data
        if (this.goalPositionPub != null) {
            long timestamp = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);

            goalPositionPub.set(new Pose2d(xGoal.position, yGoal.position, targetDirection), timestamp);
            setpointPositionPub.set(new Pose2d(xSetpoint.position, ySetpoint.position, targetDirection), timestamp);
            setpointVelocityPub.set(
                    new ChassisSpeeds(xSetpoint.velocity, ySetpoint.velocity, headingCorrectionOutput), timestamp);
            errorCorrectionVelocityPub.set(
                    new ChassisSpeeds(xCorrectionOutput, yCorrectionOutput, headingCorrectionOutput), timestamp);
            appliedVelocityPub.set(toApplyFieldSpeeds, timestamp);
        }

        return applyRobotSpeeds
                .withSpeeds(toApplyRobotSpeeds)
                .withCenterOfRotation(centerOfRotation)
                .withDriveRequestType(driveRequestType)
                .withSteerRequestType(steerRequestType)
                .withDesaturateWheelSpeeds(desaturateWheelSpeeds)
                .withWheelForceFeedforwardsX(
                        previousSwerveSetpoint.feedforwards().robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(
                        previousSwerveSetpoint.feedforwards().robotRelativeForcesYNewtons())
                .apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset the profile used for the target direction next time it is used.
     */
    public void resetRequest() {
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
    public XYHeadingAlignment withTargetPose(Pose2d newTargetPose) {
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
    public XYHeadingAlignment withCenterOfRotation(Translation2d newCenterOfRotation) {
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
    public XYHeadingAlignment withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
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
    public XYHeadingAlignment withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
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
    public XYHeadingAlignment withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.desaturateWheelSpeeds = newDesaturateWheelSpeeds;
        return this;
    }

    /**
     * Modifies the PID gains for the x and y controllers and returns itself.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @return this object
     */
    public XYHeadingAlignment withTranslationalPIDGains(double kp, double ki, double kd) {
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
    public XYHeadingAlignment withRotationalPIDGains(double kp, double ki, double kd) {
        this.headingController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public XYHeadingAlignment withHeadingTolerance(Angle toleranceAmount) {
        this.headingController.setTolerance(toleranceAmount.in(Radians));
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the x and y controllers and returns itself.
     *
     * @param toleranceAmount The maximum amount of distance the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public XYHeadingAlignment withLinearTolerance(Distance toleranceAmount) {
        this.xController.setTolerance(toleranceAmount.in(Meters));
        this.yController.setTolerance(toleranceAmount.in(Meters));
        return this;
    }
}
