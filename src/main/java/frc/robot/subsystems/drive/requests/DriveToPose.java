package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
 * and {@link com.ctre.phoenix6.swerve.utility.PhoenixPIDController PhoenixPIDControllers} for error correction on x, y, and heading.
 * <p>
 * It also has gives option of using PathPlanner's {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator SwerveSetpointGenerator}
 * to ensure that the motion respects the robot's contraints.
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
     * The desired pose to reach.
     * This pose has the blue alliance origin.
     */
    private Pose2d m_targetPose = new Pose2d();

    private final DriveWithSetpointGeneration m_drive;

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

    private boolean m_resetRequested = false;

    // NetworkTables logging
    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_table = m_inst.getTable("Swerve Requests").getSubTable("Drive to Pose");
    private final StructPublisher<Pose2d> m_goalPositionPub = m_table.getSubTable("Field-relative Goal")
            .getStructTopic("Position", Pose2d.struct)
            .publish();

    private final NetworkTable m_setpointTable = m_table.getSubTable("Field-relative Setpoint");
    private final StructPublisher<Pose2d> m_setpointPositionPub =
            m_setpointTable.getStructTopic("Position", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> m_setpointVelocityPub =
            m_setpointTable.getStructTopic("Velocity", ChassisSpeeds.struct).publish();

    private final StructPublisher<ChassisSpeeds> m_errorCorrectionVelocityPub = m_table.getStructTopic(
                    "Field-relative Error Correction Velocity", ChassisSpeeds.struct)
            .publish();

    /**
     * Creates a new profiled request with the given constraints, and without Swerve Setpoint Generator.
     *
     * @param kTranslationP The P gain for the translation controller in meters per second output per meter error.
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     * @param linearConstraints Constraints for the X and Y trapezoid profiles
     */
    public DriveToPose(
            double kTranslationP,
            double kRotationP,
            double maxAngularVelocity,
            TrapezoidProfile.Constraints linearConstraints) {
        m_drive = new DriveWithSetpointGeneration();

        m_xProfile = new TrapezoidProfile(linearConstraints);
        m_xController = new PhoenixPIDController(kTranslationP, 0.0, 0.0);
        m_yProfile = new TrapezoidProfile(linearConstraints);
        m_yController = new PhoenixPIDController(kTranslationP, 0.0, 0.0);

        m_headingController = new PhoenixPIDController(kRotationP, 0.0, 0.0);
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        m_maxAngularVelocity = Math.abs(maxAngularVelocity);
    }

    /**
     * Creates a new profiled request with the given constraints, and with Swerve Setpoint Generator.
     *
     * @param kTranslationP The P gain for the translation controller in meters per second output per meter error.
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     * @param linearConstraints Constraints for the X and Y trapezoid profiles
     * @param swerveSetpointGenerator The Swerve Setpoint Generator to use when driving.
     * @param updatePeriod The amount of time between robot updates in seconds.
     */
    public DriveToPose(
            double kTranslationP,
            double kRotationP,
            double maxAngularVelocity,
            TrapezoidProfile.Constraints linearConstraints,
            SwerveSetpointGenerator swerveSetpointGenerator,
            double updatePeriod) {
        m_drive = new DriveWithSetpointGeneration(swerveSetpointGenerator, updatePeriod);

        m_xProfile = new TrapezoidProfile(linearConstraints);
        m_xController = new PhoenixPIDController(kTranslationP, 0.0, 0.0);
        m_yProfile = new TrapezoidProfile(linearConstraints);
        m_yController = new PhoenixPIDController(kTranslationP, 0.0, 0.0);

        m_headingController = new PhoenixPIDController(kRotationP, 0.0, 0.0);
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        m_maxAngularVelocity = Math.abs(maxAngularVelocity);
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
        double toApplyX = xSetpoint.velocity + xCorrectionOutput;
        m_drive.withVelocityX(toApplyX);

        TrapezoidProfile.State ySetpoint = m_yProfile.calculate(timeSinceStart, m_yStartingState, m_yGoal);
        double yCorrectionOutput =
                m_yController.calculate(currentPose.getY(), ySetpoint.position, parameters.timestamp);
        // Must check if at setpoint after making the calculation because the error gets stored in the controller.
        if (m_yController.atSetpoint()) {
            yCorrectionOutput = 0;
        }
        double toApplyY = ySetpoint.velocity + yCorrectionOutput;
        m_drive.withVelocityY(toApplyY);

        // Calculate the extra angular velocity necessary to get the robot to the correct angle.
        double headingCorrectionOutput = m_headingController.calculate(
                currentAngle.getRadians(), targetDirection.getRadians(), parameters.timestamp);

        if (m_headingController.atSetpoint()) {
            headingCorrectionOutput = 0;
        }

        double toApplyOmega = MathUtil.clamp(headingCorrectionOutput, -m_maxAngularVelocity, m_maxAngularVelocity);
        m_drive.withRotationalRate(toApplyOmega);

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

        return m_drive.apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset the profile used for the target direction next time it is used.
     */
    public void resetRequest() {
        m_drive.resetRequest();
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
        m_drive.withCenterOfRotation(newCenterOfRotation);
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
        m_drive.withDriveRequestType(newDriveRequestType);
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
        m_drive.withSteerRequestType(newSteerRequestType);
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
