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
 * <p>
 * This swerve request is based on {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle},
 * and makes some other improvements besides the motion profiles.
 * <p>
 * Important: You may not change the targetPose while the swerve request is in use.
 */
public class ProfiledXYHeadingAlignment implements ResettableSwerveRequest {
    /**
     * The field-centric chassis speeds to apply to the drivetrain.
     */
    private final ChassisSpeeds toApplySpeeds = new ChassisSpeeds();
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

    private final ApplyFieldSpeeds applyFieldSpeeds =
            new ApplyFieldSpeeds().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    // X position profile and PID controller
    private final TrapezoidProfile xProfile;
    private final TrapezoidProfile.State xStartingState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State xGoal = new TrapezoidProfile.State();
    private final PhoenixPIDController xController = new PhoenixPIDController(0, 0, 0);

    // Y position profile and PID controller
    private final TrapezoidProfile yProfile;
    private final TrapezoidProfile.State yStartingState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State yGoal = new TrapezoidProfile.State();
    private final PhoenixPIDController yController = new PhoenixPIDController(0, 0, 0);

    // Rotation PID controller
    private final PhoenixPIDController headingController;
    private final double maxAngularVelocity;

    private double startingTimestamp = 0;

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
     * @param kp The P gain for the heading controller in radians per second output per radian error.
     * @param ki The I gain for the heading controller in radians per second output per integral of radian error.
     * @param kp The P gain for the heading controller in radians per second output per the derivative of error radians per second.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     * @param linearConstraints Constraints for the X and Y trapezoid profiles
     */
    public ProfiledXYHeadingAlignment(
            double kp,
            double ki,
            double kd,
            double maxAngularVelocity,
            TrapezoidProfile.Constraints linearConstraints) {
        xProfile = new TrapezoidProfile(linearConstraints);
        yProfile = new TrapezoidProfile(linearConstraints);

        headingController = new PhoenixPIDController(kp, ki, kd);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.maxAngularVelocity = maxAngularVelocity;

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
     * Creates a new profiled request with the given constraints,
     * and logs data to "X Y Heading Alignment"
     *
     * @param kp The P gain for the heading controller in radians per second output per radian error.
     * @param ki The I gain for the heading controller in radians per second output per integral of radian error.
     * @param kp The P gain for the heading controller in radians per second output per the derivative of error radians per second.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     * @param linearConstraints Constraints for the X and Y trapezoid profiles
     * @param loggingPath The NetworkTable to log data into.
     */
    public ProfiledXYHeadingAlignment(
            double kp,
            double ki,
            double kd,
            double maxAngularVelocity,
            TrapezoidProfile.Constraints linearConstraints,
            NetworkTable loggingPath) {
        xProfile = new TrapezoidProfile(linearConstraints);
        yProfile = new TrapezoidProfile(linearConstraints);

        headingController = new PhoenixPIDController(kp, ki, kd);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.maxAngularVelocity = maxAngularVelocity;

        // Make PID gains tunable from NetworkTables
        SmartDashboard.putData("X Controller", xController);
        SmartDashboard.putData("Y Controller", yController);
        SmartDashboard.putData("Heading Controller", headingController);

        NetworkTable motionTable = loggingPath.getSubTable("X Y Heading Alignment");
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
        Rotation2d targetDirection = targetPose.getRotation();

        if (resetRequested) {
            xStartingState.position = currentPose.getX();
            xStartingState.velocity = parameters.currentChassisSpeed.vxMetersPerSecond;
            yStartingState.position = currentPose.getY();
            yStartingState.velocity = parameters.currentChassisSpeed.vyMetersPerSecond;
            startingTimestamp = parameters.timestamp;
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
        toApplySpeeds.vxMetersPerSecond = xSetpoint.velocity + xCorrectionOutput;

        TrapezoidProfile.State ySetpoint = yProfile.calculate(time, yStartingState, yGoal);
        double yCorrectionOutput = yController.calculate(currentPose.getY(), ySetpoint.position, parameters.timestamp);
        if (yController.atSetpoint()) {
            yCorrectionOutput = 0;
        }
        toApplySpeeds.vyMetersPerSecond = ySetpoint.velocity + yCorrectionOutput;

        // Calculate the extra angular velocity necessary to get the robot to the correct angle.
        double headingCorrectionOutput = headingController.calculate(
                currentAngle.getRadians(), targetDirection.getRadians(), parameters.timestamp);

        if (headingController.atSetpoint()) {
            headingCorrectionOutput = 0;
        }

        toApplySpeeds.omegaRadiansPerSecond =
                MathUtil.clamp(headingCorrectionOutput, -maxAngularVelocity, maxAngularVelocity);

        // If one of the publishers isn't null, all of them were initialized, so log data
        if (this.goalPositionPub != null) {
            long timestamp = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);

            goalPositionPub.set(new Pose2d(xGoal.position, yGoal.position, targetDirection), timestamp);
            setpointPositionPub.set(new Pose2d(xSetpoint.position, ySetpoint.position, targetDirection), timestamp);
            setpointVelocityPub.set(
                    new ChassisSpeeds(xSetpoint.velocity, ySetpoint.velocity, headingCorrectionOutput), timestamp);
            errorCorrectionVelocityPub.set(
                    new ChassisSpeeds(xCorrectionOutput, yCorrectionOutput, headingCorrectionOutput), timestamp);
            appliedVelocityPub.set(toApplySpeeds, timestamp);
        }

        return applyFieldSpeeds
                .withSpeeds(toApplySpeeds)
                .withCenterOfRotation(centerOfRotation)
                .withDriveRequestType(driveRequestType)
                .withSteerRequestType(steerRequestType)
                .withDesaturateWheelSpeeds(desaturateWheelSpeeds)
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

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public ProfiledXYHeadingAlignment withHeadingTolerance(Angle toleranceAmount) {
        this.headingController.setTolerance(toleranceAmount.in(Radians));
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the x and y controllers and returns itself.
     *
     * @param toleranceAmount The maximum amount of distance the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public ProfiledXYHeadingAlignment withLinearTolerance(Distance toleranceAmount) {
        this.xController.setTolerance(toleranceAmount.in(Meters));
        this.yController.setTolerance(toleranceAmount.in(Meters));
        return this;
    }
}
