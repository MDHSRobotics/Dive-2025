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
 * This request currently supports drivetrains with 4 modules.
 * <p>
 * Side note: In the future, this could be improved with a more aggressive {@link edu.wpi.first.math.trajectory.ExponentialProfile exponential profile}
 * because the {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator SwerveSetpointGenerator}
 * ensures that the motion respects the robot's constraints.
 * However, this would require accurate kV and kA gains from drive SysId, which are hard to get with <a href="https://www.vexrobotics.com/colsonperforma.html">Colson wheels</a>.
 */
public class DriveToPose implements ResettableSwerveRequest {
    /**
     * The robot-relative chassis speeds to apply to the drivetrain.
     */
    private ChassisSpeeds m_toApplyRobotSpeeds = new ChassisSpeeds();
    /**
     * The field-relative chassis speeds to log to NetworkTables.
     */
    private ChassisSpeeds m_toApplyFieldSpeeds = new ChassisSpeeds();
    /**
     * The desired pose to reach.
     * This pose has the blue alliance origin.
     */
    private Pose2d m_targetPose = new Pose2d();

    /** Must be robot-relative because the swerve setpoint generator outputs robot-relative wheel forces */
    private final ApplyRobotSpeeds m_applyRobotSpeeds = new ApplyRobotSpeeds().withDesaturateWheelSpeeds(false);

    // X position profile and PID controller
    private final TrapezoidProfile m_xProfile;
    private final TrapezoidProfile.State m_xStartingState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State m_xGoal = new TrapezoidProfile.State();
    private final PhoenixPIDController m_xController;

    // Y position profile and PID controller
    private final TrapezoidProfile m_yProfile;
    private final TrapezoidProfile.State m_yStartingState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State m_yGoal = new TrapezoidProfile.State();
    private final PhoenixPIDController m_yController;

    // Rotation PID controller
    private final PhoenixPIDController m_headingController;
    private final double m_maxAngularVelocity;

    /**
     * The timestamp for the start of this request, in the timebase of {@link Utils#getCurrentTimeSeconds()}.
     * This is used for the trapezoid profiles.
     */
    private double m_profileStartingTimestamp = 0;

    private final SwerveSetpointGenerator m_setpointGenerator;
    private SwerveModuleState[] m_startingModuleStates = new SwerveModuleState[4];
    private SwerveSetpoint m_previousSwerveSetpoint;
    /**
     * The update period for the {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator swerve setpoint generator} in seconds.
     */
    private final double m_updatePeriod;

    private boolean m_resetRequested = false;

    // NetworkTables logging
    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_table = m_inst.getTable("Swerve Requests").getSubTable("Drive to Pose");
    private final StructPublisher<Pose2d> m_goalPositionPub = m_table.getSubTable("Goal")
            .getStructTopic("Position", Pose2d.struct)
            .publish();

    private final NetworkTable m_setpointTable = m_table.getSubTable("Setpoint");
    private final StructPublisher<Pose2d> m_setpointPositionPub =
            m_setpointTable.getStructTopic("Position", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> m_setpointVelocityPub =
            m_setpointTable.getStructTopic("Velocity", ChassisSpeeds.struct).publish();

    private final StructPublisher<ChassisSpeeds> m_errorCorrectionVelocityPub = m_table.getStructTopic(
                    "Error Correction Velocity", ChassisSpeeds.struct)
            .publish();
    private final StructPublisher<ChassisSpeeds> m_appliedVelocityPub =
            m_table.getStructTopic("Applied Velocity", ChassisSpeeds.struct).publish();

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
    public DriveToPose(
            double kTranslationP,
            double kRotationP,
            double maxAngularVelocity,
            TrapezoidProfile.Constraints linearConstraints,
            RobotConfig robotConfig,
            double maxSteerVelocityRadsPerSec,
            double updatePeriod) {
        m_xProfile = new TrapezoidProfile(linearConstraints);
        m_xController = new PhoenixPIDController(kTranslationP, 0.0, 0.0);
        m_yProfile = new TrapezoidProfile(linearConstraints);
        m_yController = new PhoenixPIDController(kTranslationP, 0.0, 0.0);

        m_headingController = new PhoenixPIDController(kRotationP, 0.0, 0.0);
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        m_maxAngularVelocity = maxAngularVelocity;

        m_setpointGenerator = new SwerveSetpointGenerator(robotConfig, maxSteerVelocityRadsPerSec);
        m_updatePeriod = updatePeriod;
    }

    /**
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double, double)
     * @see edu.wpi.first.math.controller.ProfiledPIDController#calculate(double)
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Pose2d currentPose = parameters.currentPose;
        Rotation2d currentAngle = parameters.currentPose.getRotation();
        Rotation2d targetDirection = m_targetPose.getRotation();

        if (m_resetRequested) {
            m_xStartingState.position = currentPose.getX();
            m_xStartingState.velocity = parameters.currentChassisSpeed.vxMetersPerSecond;
            m_yStartingState.position = currentPose.getY();
            m_yStartingState.velocity = parameters.currentChassisSpeed.vyMetersPerSecond;
            m_profileStartingTimestamp = parameters.timestamp;

            for (int i = 0; i < 4; ++i) {
                m_startingModuleStates[i] = modulesToApply[i].getCurrentState();
            }
            m_previousSwerveSetpoint = new SwerveSetpoint(
                    parameters.currentChassisSpeed, m_startingModuleStates, DriveFeedforwards.zeros(4));

            m_xController.reset();
            m_yController.reset();
            m_headingController.reset();
            m_resetRequested = false;
        }

        double timeSinceStart = parameters.timestamp - m_profileStartingTimestamp;

        TrapezoidProfile.State xSetpoint = m_xProfile.calculate(timeSinceStart, m_xStartingState, m_xGoal);
        double xCorrectionOutput =
                m_xController.calculate(currentPose.getX(), xSetpoint.position, parameters.timestamp);
        // Must check if at setpoint after making the calculation because the error gets stored in the controller.
        if (m_xController.atSetpoint()) {
            xCorrectionOutput = 0;
        }
        m_toApplyFieldSpeeds.vxMetersPerSecond = xSetpoint.velocity + xCorrectionOutput;

        TrapezoidProfile.State ySetpoint = m_yProfile.calculate(timeSinceStart, m_yStartingState, m_yGoal);
        double yCorrectionOutput =
                m_yController.calculate(currentPose.getY(), ySetpoint.position, parameters.timestamp);
        // Must check if at setpoint after making the calculation because the error gets stored in the controller.
        if (m_yController.atSetpoint()) {
            yCorrectionOutput = 0;
        }
        m_toApplyFieldSpeeds.vyMetersPerSecond = ySetpoint.velocity + yCorrectionOutput;

        // Calculate the extra angular velocity necessary to get the robot to the correct angle.
        double headingCorrectionOutput = m_headingController.calculate(
                currentAngle.getRadians(), targetDirection.getRadians(), parameters.timestamp);

        if (m_headingController.atSetpoint()) {
            headingCorrectionOutput = 0;
        }

        m_toApplyFieldSpeeds.omegaRadiansPerSecond =
                MathUtil.clamp(headingCorrectionOutput, -m_maxAngularVelocity, m_maxAngularVelocity);

        // The generator requires robot-relative speeds
        m_toApplyRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_toApplyFieldSpeeds, currentAngle);

        // Improve the motion profile generated movement with a setpoint that respects the robot's constraints better.
        m_previousSwerveSetpoint =
                m_setpointGenerator.generateSetpoint(m_previousSwerveSetpoint, m_toApplyRobotSpeeds, m_updatePeriod);

        m_toApplyRobotSpeeds = m_previousSwerveSetpoint.robotRelativeSpeeds();

        // Convert back to field-relative speeds for the sake of easier logging.
        m_toApplyFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_toApplyRobotSpeeds, currentAngle);

        // NetworkTables logging
        long timestampMicroseconds = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);

        m_goalPositionPub.set(new Pose2d(m_xGoal.position, m_yGoal.position, targetDirection), timestampMicroseconds);
        m_setpointPositionPub.set(
                new Pose2d(xSetpoint.position, ySetpoint.position, targetDirection), timestampMicroseconds);
        m_setpointVelocityPub.set(
                new ChassisSpeeds(xSetpoint.velocity, ySetpoint.velocity, headingCorrectionOutput),
                timestampMicroseconds);
        m_errorCorrectionVelocityPub.set(
                new ChassisSpeeds(xCorrectionOutput, yCorrectionOutput, headingCorrectionOutput),
                timestampMicroseconds);
        m_appliedVelocityPub.set(m_toApplyFieldSpeeds, timestampMicroseconds);

        return m_applyRobotSpeeds
                .withSpeeds(m_toApplyRobotSpeeds)
                .withWheelForceFeedforwardsX(
                        m_previousSwerveSetpoint.feedforwards().robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(
                        m_previousSwerveSetpoint.feedforwards().robotRelativeForcesYNewtons())
                .apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset the profile used for the target direction next time it is used.
     */
    public void resetRequest() {
        m_resetRequested = true;
    }

    /**
     * Modifies the TargetPose parameter and returns itself.
     * <p>
     * The desired position and direction to face.
     * This is in blue alliance origin.
     *
     * @param newTargetPose Parameter to modify
     * @return this object
     */
    public DriveToPose withTargetPose(Pose2d newTargetPose) {
        m_targetPose = newTargetPose;
        m_xGoal.position = newTargetPose.getX();
        m_yGoal.position = newTargetPose.getY();
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
    public DriveToPose withCenterOfRotation(Translation2d newCenterOfRotation) {
        m_applyRobotSpeeds.withCenterOfRotation(newCenterOfRotation);
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
    public DriveToPose withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        m_applyRobotSpeeds.withDriveRequestType(newDriveRequestType);
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
    public DriveToPose withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        m_applyRobotSpeeds.withSteerRequestType(newSteerRequestType);
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
    public DriveToPose withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        m_applyRobotSpeeds.withDesaturateWheelSpeeds(newDesaturateWheelSpeeds);
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
    public DriveToPose withTranslationalPIDGains(double kp, double ki, double kd) {
        m_xController.setPID(kp, ki, kd);
        m_yController.setPID(kp, ki, kd);
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
    public DriveToPose withRotationalPIDGains(double kp, double ki, double kd) {
        m_headingController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public DriveToPose withHeadingTolerance(Angle toleranceAmount) {
        m_headingController.setTolerance(toleranceAmount.in(Radians));
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the x and y controllers and returns itself.
     *
     * @param toleranceAmount The maximum amount of distance the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public DriveToPose withLinearTolerance(Distance toleranceAmount) {
        m_xController.setTolerance(toleranceAmount.in(Meters));
        m_yController.setTolerance(toleranceAmount.in(Meters));
        return this;
    }
}
