package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.config.RobotConfig;
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
 * using PathPlanner's {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator SwerveSetpointGenerator}
 * to ensure that the motion respects the robot's contraints.
 */
public class DriveWithSetpointGeneration implements ResettableSwerveRequest {
    /**
     * The robot-relative chassis speeds to apply to the drivetrain.
     */
    private ChassisSpeeds m_toApplyRobotSpeeds = new ChassisSpeeds();
    /**
     * The field-relative chassis speeds to accept field-relative velocities from the user.
     */
    private ChassisSpeeds m_toApplyFieldSpeeds = new ChassisSpeeds();
    /**
     * The allowable deadband of the request, in m/s.
     */
    public double m_deadband = 0;
    /**
     * The rotational deadband of the request, in radians per second.
     */
    public double m_rotationalDeadband = 0;
    /**
     * The perspective to use when determining which direction is forward.
     */
    public ForwardPerspectiveValue m_drivingPerspective = ForwardPerspectiveValue.OperatorPerspective;

    // DesaturateWheelSpeeds should be set to false because Swerve Setpoint Generator desaturates wheel speeds for you.
    private final ApplyRobotSpeeds m_applyRobotSpeeds = new ApplyRobotSpeeds().withDesaturateWheelSpeeds(false);

    private boolean m_resetRequested = false;

    // Swerve Setpoint Generator
    private final SwerveSetpointGenerator m_setpointGenerator;
    private SwerveModuleState[] m_startingModuleStates = new SwerveModuleState[4];
    private SwerveSetpoint m_previousSwerveSetpoint;
    /**
     * The update period for the {@link com.pathplanner.lib.util.swerve.SwerveSetpointGenerator swerve setpoint generator} in seconds.
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
     * Creates a new request with the swerve setpoint generator configuration.
     *
     * @param robotConfig The PathPlanner config for the robot
     * @param maxSteerVelocityRadsPerSec The maximum rotation velocity of a swerve module, in radians per second
     * @param updatePeriod The amount of time between robot updates in seconds.
     */
    public DriveWithSetpointGeneration(
            RobotConfig robotConfig, double maxSteerVelocityRadsPerSec, double updatePeriod) {
        m_setpointGenerator = new SwerveSetpointGenerator(robotConfig, maxSteerVelocityRadsPerSec);
        m_startingModuleStates = new SwerveModuleState[4];
        m_updatePeriod = updatePeriod;
    }

    /**
     * @see com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric#apply(com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.LegacySwerveControlRequestParameters, com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule...)
     */
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        final ChassisSpeeds toApplyFieldSpeeds = new ChassisSpeeds(
                m_toApplyFieldSpeeds.vxMetersPerSecond,
                m_toApplyFieldSpeeds.vyMetersPerSecond,
                m_toApplyFieldSpeeds.omegaRadiansPerSecond);

        if (m_resetRequested) {
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
        m_toApplyRobotSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(toApplyFieldSpeeds, parameters.currentPose.getRotation());

        // NetworkTables logging (must be done before generating the setpoint)
        long timestampMicroseconds = DriveTelemetry.stateTimestampToNTTimestamp(parameters.timestamp);
        m_requestedSpeedsPub.set(m_toApplyRobotSpeeds, timestampMicroseconds);

        // Improve the profiled movement with a setpoint that respects the robot's constraints better.
        m_previousSwerveSetpoint =
                m_setpointGenerator.generateSetpoint(m_previousSwerveSetpoint, m_toApplyRobotSpeeds, m_updatePeriod);

        m_toApplyRobotSpeeds = m_previousSwerveSetpoint.robotRelativeSpeeds();

        // More NetworkTables logging
        m_appliedSpeedsPub.set(m_toApplyRobotSpeeds);

        return m_applyRobotSpeeds
                .withSpeeds(m_toApplyRobotSpeeds)
                .withWheelForceFeedforwardsX(
                        m_previousSwerveSetpoint.feedforwards().robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(
                        m_previousSwerveSetpoint.feedforwards().robotRelativeForcesYNewtons())
                .apply(parameters, modulesToApply);
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
