package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
 * It also has gives option of using PathPlanner's {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator SwerveSetpointGenerator}
 * to ensure that the motion respects the robot's contraints.
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
    private Rotation2d m_targetDirection = new Rotation2d();

    /**
     * The perspective to use when determing which direction is forward for aiming.
     * By default, this swerve request considers the angle to be in <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">the blue coordinate system.</a>
     */
    private ForwardPerspectiveValue m_aimingPerspective = ForwardPerspectiveValue.BlueAlliance;

    private final DriveWithSetpointGeneration m_drive;

    private final PhoenixPIDController m_headingController;
    private final double m_maxAngularVelocity;

    private boolean m_resetRequested = false;
    private boolean m_motionIsFinished = false;

    // NetworkTables logging
    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_table = m_inst.getTable("Swerve Requests").getSubTable("Drive Facing Angle");
    private final DoublePublisher m_goalPositionPub =
            m_table.getSubTable("Goal").getDoubleTopic("Position (radians)").publish();
    private final BooleanPublisher m_motionIsFinishedPub =
            m_table.getBooleanTopic("Motion is Finished").publish();

    /**
     * Creates a new request with the given gains, and without Swerve Setpoint Generation.
     *
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     */
    public DriveFacingAngle(double kRotationP, double maxAngularVelocity) {
        m_drive = new DriveWithSetpointGeneration();

        m_headingController = new PhoenixPIDController(kRotationP, 0.0, 0.0);
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        m_maxAngularVelocity = Math.abs(maxAngularVelocity);
    }

    /**
     * Creates a new request with the given gains, and with Swerve Setpoint Generation.
     *
     * @param kRotationP The P gain for the heading controller in radians per second output per radian error.
     * @param maxAngularVelocity The angular velocity to clamp the heading controller output with (in radians per second).
     * @param swerveSetpointGenerator The Swerve Setpoint Generator to use when driving.
     * @param updatePeriod The amount of time between robot updates in seconds.
     */
    public DriveFacingAngle(
            double kRotationP,
            double maxAngularVelocity,
            SwerveSetpointGenerator swerveSetpointGenerator,
            double updatePeriod) {
        m_drive = new DriveWithSetpointGeneration(swerveSetpointGenerator, updatePeriod);

        m_headingController = new PhoenixPIDController(kRotationP, 0.0, 0.0);
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        m_maxAngularVelocity = Math.abs(maxAngularVelocity);
    }

    /**
     * @see com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle#apply(SwerveControlParameters, SwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Rotation2d blueTargetDirection = m_targetDirection;

        if (m_resetRequested) {
            m_headingController.reset();
            m_resetRequested = false;
        }

        /* If the user requested a target direction according to the operator perspective, rotate our target direction by the angle */
        if (m_aimingPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            blueTargetDirection = m_targetDirection.rotateBy(parameters.operatorForwardDirection);
        }

        double toApplyOmega = m_headingController.calculate(
                parameters.currentPose.getRotation().getRadians(),
                blueTargetDirection.getRadians(),
                parameters.timestamp);

        toApplyOmega = MathUtil.clamp(toApplyOmega, -m_maxAngularVelocity, m_maxAngularVelocity);

        if (m_headingController.atSetpoint()) {
            toApplyOmega = 0;
            m_motionIsFinished = true;
        }

        // NetworkTables logging
        long timestampMicroseconds = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);

        m_goalPositionPub.set(blueTargetDirection.getRadians(), timestampMicroseconds);
        m_motionIsFinishedPub.set(m_motionIsFinished, timestampMicroseconds);

        return m_drive.withRotationalRate(toApplyOmega).apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset the profile used for the target direction next time it is used.
     */
    public void resetRequest() {
        m_drive.resetRequest();
        m_resetRequested = true;
        m_motionIsFinished = false;
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
        m_drive.withVelocityX(newVelocityX);
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
        m_drive.withVelocityX(newVelocityX);
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
        m_drive.withVelocityY(newVelocityY);
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
        m_drive.withVelocityY(newVelocityY);
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
        m_targetDirection = newTargetDirection;
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
        m_drive.withDeadband(newDeadband);
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
        m_drive.withDeadband(newDeadband);
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
    public DriveFacingAngle withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
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
    public DriveFacingAngle withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        m_drive.withSteerRequestType(newSteerRequestType);
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
        m_drive.withDrivingPerspective(newDrivingPerspective);
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
        m_aimingPerspective = newAimingPerspective;
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
        m_headingController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Modifies the setpoint tolerance for the heading controller and returns itself.
     *
     * @param toleranceAmount The maximum amount of degrees or radians the robot can be from its goal when calling atSetpoint().
     * @return this object
     */
    public DriveFacingAngle withTolerance(Angle toleranceAmount) {
        m_headingController.setTolerance(toleranceAmount.in(Radians));
        return this;
    }

    /**
     * @return Whether or not the robot has reached its target rotation,
     * based on the tolerance set using {@link frc.robot.subsystems.drive.requests.DriveFacingAngle#withTolerance(Angle) withTolerance()}
     */
    public boolean motionIsFinished() {
        return m_motionIsFinished;
    }
}
