package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.DriveTelemetry;

/**
 * Drives the swerve drivetrain in a field-centric manner,
 * with the option of using PathPlanner's {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator SwerveSetpointGenerator}
 * to ensure that the motion respects the robot's contraints.
 * <p>
 * Note: If the setpoint generator is not used,
 * the "Requested Robot-relative Speeds" and "Applied Robot-relative Speeds" NetworkTables publishers will display the same speeds.
 */
public class DriveWithSetpointGeneration implements ResettableSwerveRequest {
    /**
     * The field-relative chassis speeds to accept field-relative velocities from the user.
     */
    private ChassisSpeeds m_toApplyFieldSpeeds = new ChassisSpeeds();
    /**
     * The allowable deadband of the request, in m/s.
     */
    private double m_deadband = 0;
    /**
     * The rotational deadband of the request, in radians per second.
     */
    private double m_rotationalDeadband = 0;
    /**
     * The perspective to use when determining which direction is forward.
     */
    private ForwardPerspectiveValue m_drivingPerspective = ForwardPerspectiveValue.OperatorPerspective;

    private final ApplyRobotSpeeds m_applyRobotSpeeds = new ApplyRobotSpeeds();

    private boolean m_resetRequested = false;

    // Swerve Setpoint Generator
    private final SwerveSetpointGenerator m_setpointGenerator;
    private SwerveModuleState[] m_startingModuleStates;
    private SwerveSetpoint m_previousSwerveSetpoint;
    /**
     * The update period for the {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator Swerve Setpoint Generator} in seconds.
     */
    private final double m_updatePeriod;

    // NetworkTables logging
    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_table =
            m_inst.getTable("Swerve Requests").getSubTable("Drive With Swerve Setpoint Generator");
    private final StructPublisher<ChassisSpeeds> m_requestedSpeedsPub = m_table.getStructTopic(
                    "Requested Robot-relative Speeds", ChassisSpeeds.struct)
            .publish();
    private final StructPublisher<ChassisSpeeds> m_appliedSpeedsPub = m_table.getStructTopic(
                    "Applied Robot-relative Speeds", ChassisSpeeds.struct)
            .publish();

    /**
     * Creates a new request without the Swerve Setpoint Generator.
     */
    public DriveWithSetpointGeneration() {
        m_setpointGenerator = null;
        m_startingModuleStates = null;
        m_updatePeriod = 0.0;
    }

    /**
     * Creates a new request with the Swerve Setpoint Generator.
     *
     * @param swerveSetpointGenerator The Swerve Setpoint Generator to use when driving.
     * @param updatePeriod The amount of time between robot updates in seconds.
     */
    public DriveWithSetpointGeneration(SwerveSetpointGenerator swerveSetpointGenerator, double updatePeriod) {
        m_setpointGenerator = swerveSetpointGenerator;
        m_startingModuleStates = new SwerveModuleState[4];
        m_updatePeriod = updatePeriod;

        // This should be set to false because Swerve Setpoint Generator desaturates wheel speeds for you.
        m_applyRobotSpeeds.withDesaturateWheelSpeeds(false);
    }

    /**
     * @see com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric#apply(com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.LegacySwerveControlRequestParameters, com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        // Clone the field speeds so we can modify them every time without affecting the speeds requested by the user.
        final ChassisSpeeds toApplyFieldSpeeds = new ChassisSpeeds(
                m_toApplyFieldSpeeds.vxMetersPerSecond,
                m_toApplyFieldSpeeds.vyMetersPerSecond,
                m_toApplyFieldSpeeds.omegaRadiansPerSecond);

        // Resets are only required if the Swerve Setpoint Generator is in use.
        if (m_resetRequested && m_setpointGenerator != null) {
            for (int i = 0; i < 4; ++i) {
                m_startingModuleStates[i] = modulesToApply[i].getCurrentState();
            }
            m_previousSwerveSetpoint = new SwerveSetpoint(
                    parameters.currentChassisSpeed, m_startingModuleStates, DriveFeedforwards.zeros(4));
            m_resetRequested = false;
        }

        /* If the user requested to drive according to the operator perspective, rotate the velocities by the angle */
        if (m_drivingPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            final Translation2d rotatedVelocities = new Translation2d(
                            toApplyFieldSpeeds.vxMetersPerSecond, toApplyFieldSpeeds.vyMetersPerSecond)
                    .rotateBy(parameters.operatorForwardDirection);
            toApplyFieldSpeeds.vxMetersPerSecond = rotatedVelocities.getX();
            toApplyFieldSpeeds.vyMetersPerSecond = rotatedVelocities.getY();
        }

        // Apply deadbands
        if (Math.hypot(toApplyFieldSpeeds.vxMetersPerSecond, toApplyFieldSpeeds.vyMetersPerSecond) < m_deadband) {
            toApplyFieldSpeeds.vxMetersPerSecond = 0.0;
            toApplyFieldSpeeds.vyMetersPerSecond = 0.0;
        }
        if (Math.abs(toApplyFieldSpeeds.omegaRadiansPerSecond) < m_rotationalDeadband) {
            toApplyFieldSpeeds.omegaRadiansPerSecond = 0.0;
        }

        // The generator requires robot-relative speeds, so we always convert the field-relative input to
        // robot-relative.
        ChassisSpeeds toApplyRobotSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(toApplyFieldSpeeds, parameters.currentPose.getRotation());

        // NetworkTables logging (must be done before generating the setpoint)
        long timestampMicroseconds = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);
        m_requestedSpeedsPub.set(toApplyRobotSpeeds, timestampMicroseconds);

        // If the setpoint generator is configured, improve the profiled movement with a setpoint that
        // respects the robot's constraints better.
        if (m_setpointGenerator != null) {
            m_previousSwerveSetpoint =
                    m_setpointGenerator.generateSetpoint(m_previousSwerveSetpoint, toApplyRobotSpeeds, m_updatePeriod);

            toApplyRobotSpeeds = m_previousSwerveSetpoint.robotRelativeSpeeds();

            m_applyRobotSpeeds
                    .withWheelForceFeedforwardsX(
                            m_previousSwerveSetpoint.feedforwards().robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(
                            m_previousSwerveSetpoint.feedforwards().robotRelativeForcesYNewtons());
        }

        // More NetworkTables logging
        m_appliedSpeedsPub.set(toApplyRobotSpeeds);

        return m_applyRobotSpeeds.withSpeeds(toApplyRobotSpeeds).apply(parameters, modulesToApply);
    }

    /**
     * Tells the swerve request to reset next time it is used.
     */
    public void resetRequest() {
        m_resetRequested = true;
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
    public DriveWithSetpointGeneration withVelocityX(double newVelocityX) {
        m_toApplyFieldSpeeds.vxMetersPerSecond = newVelocityX;
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
    public DriveWithSetpointGeneration withVelocityX(LinearVelocity newVelocityX) {
        m_toApplyFieldSpeeds.vxMetersPerSecond = newVelocityX.in(MetersPerSecond);
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
    public DriveWithSetpointGeneration withVelocityY(double newVelocityY) {
        m_toApplyFieldSpeeds.vyMetersPerSecond = newVelocityY;
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
    public DriveWithSetpointGeneration withVelocityY(LinearVelocity newVelocityY) {
        m_toApplyFieldSpeeds.vyMetersPerSecond = newVelocityY.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the RotationalRate parameter and returns itself.
     * <p>
     * The angular rate to rotate at, in radians per second. Angular rate is defined
     * as counterclockwise positive, so this determines how fast to turn
     * counterclockwise.
     *
     * @param newRotationalRate Parameter to modify
     * @return this object
     */
    public DriveWithSetpointGeneration withRotationalRate(double newRotationalRate) {
        m_toApplyFieldSpeeds.omegaRadiansPerSecond = newRotationalRate;
        return this;
    }

    /**
     * Modifies the RotationalRate parameter and returns itself.
     * <p>
     * The angular rate to rotate at, in radians per second. Angular rate is defined
     * as counterclockwise positive, so this determines how fast to turn
     * counterclockwise.
     *
     * @param newRotationalRate Parameter to modify
     * @return this object
     */
    public DriveWithSetpointGeneration withRotationalRate(AngularVelocity newRotationalRate) {
        m_toApplyFieldSpeeds.omegaRadiansPerSecond = newRotationalRate.in(RadiansPerSecond);
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
    public DriveWithSetpointGeneration withDeadband(double newDeadband) {
        m_deadband = newDeadband;
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
    public DriveWithSetpointGeneration withDeadband(LinearVelocity newDeadband) {
        m_deadband = newDeadband.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public DriveWithSetpointGeneration withRotationalDeadband(double newRotationalDeadband) {
        m_rotationalDeadband = newRotationalDeadband;
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public DriveWithSetpointGeneration withRotationalDeadband(AngularVelocity newRotationalDeadband) {
        m_rotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
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
    public DriveWithSetpointGeneration withCenterOfRotation(Translation2d newCenterOfRotation) {
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
    public DriveWithSetpointGeneration withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
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
    public DriveWithSetpointGeneration withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        m_applyRobotSpeeds.withSteerRequestType(newSteerRequestType);
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
    public DriveWithSetpointGeneration withDrivingPerspective(ForwardPerspectiveValue newDrivingPerspective) {
        m_drivingPerspective = newDrivingPerspective;
        return this;
    }
}
