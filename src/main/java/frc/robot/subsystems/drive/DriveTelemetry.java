package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class DriveTelemetry {
    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    /**
     * Provides the robot orientation to the front limelight for <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2">megatag2</a>.
     * The LimelightHelpers equivalent to this is {@link frc.robot.util.LimelightHelpers#SetRobotOrientation(String, double, double, double, double, double, double) SetRobotOrientation()}.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">NetworkTables API documentation</a>
     */
    private final DoubleArrayPublisher m_megatag2FrontUpdater = m_inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish(PubSubOption.periodic(0.004));

    private final DoubleArrayPublisher m_megatag2BackUpdater = m_inst.getTable(VisionConstants.BACK_LIMELIGHT_NAME)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish(PubSubOption.periodic(0.004));

    /** Limelight requires this to be an array of size 6. */
    private double[] m_megatag2Orientation = new double[6];

    /* Robot swerve drive state */
    private final NetworkTable m_driveStateTable = m_inst.getTable("DriveState");
    private final StructPublisher<Pose2d> m_drivePosePub = m_driveStateTable
            .getSubTable("Poses")
            .getStructTopic("Pose", Pose2d.struct)
            .publish();
    private final StructPublisher<ChassisSpeeds> m_speedsPub = m_driveStateTable
            .getStructTopic("Field Speeds", ChassisSpeeds.struct)
            .publish();
    private final StructArrayPublisher<SwerveModuleState> m_moduleStatesPub = m_driveStateTable
            .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
            .publish();
    private final StructArrayPublisher<SwerveModuleState> m_moduleTargetsPub = m_driveStateTable
            .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
            .publish();
    private final StructArrayPublisher<SwerveModulePosition> m_modulePositionsPub = m_driveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
            .publish();
    private final DoublePublisher m_odometryFrequencyPub =
            m_driveStateTable.getDoubleTopic("OdometryFrequency").publish();
    private final DoublePublisher m_odometryPeriodPub =
            m_driveStateTable.getDoubleTopic("OdometryPeriod").publish();
    private final DoublePublisher m_linearSpeedPub =
            m_driveStateTable.getDoubleTopic("Linear Speed").publish();

    /** Accept the swerve drive state and log it to NetworkTables and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        long timestampMicroseconds = stateTimestampToNTTimestamp(state.Timestamp);

        /* Send the robot orientation to the limelight for megatag2 */
        m_megatag2Orientation[0] = state.Pose.getRotation().getDegrees();
        m_megatag2Orientation[1] = state.Speeds.omegaRadiansPerSecond * 180.0 / Math.PI;
        m_megatag2FrontUpdater.set(m_megatag2Orientation, timestampMicroseconds);
        m_megatag2BackUpdater.set(m_megatag2Orientation, timestampMicroseconds);
        // Flushing is ESSENTIAL for the limelight to receive accurate yaw and give accurate pose estimates.
        m_inst.flush();

        /* Telemeterize the swerve drive state */
        m_drivePosePub.set(state.Pose, timestampMicroseconds);
        m_speedsPub.set(
                ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation()), timestampMicroseconds);
        m_moduleStatesPub.set(state.ModuleStates, timestampMicroseconds);
        m_moduleTargetsPub.set(state.ModuleTargets, timestampMicroseconds);
        m_modulePositionsPub.set(state.ModulePositions, timestampMicroseconds);
        m_odometryFrequencyPub.set(1.0 / state.OdometryPeriod, timestampMicroseconds);
        m_odometryPeriodPub.set(state.OdometryPeriod, timestampMicroseconds);

        double linearSpeed = Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);
        m_linearSpeedPub.set(linearSpeed, timestampMicroseconds);
    }

    /**
     * Converts a {@link com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState SwerveDriveState}
     * timestamp to the timebase required for NetworkTables.
     * <p>
     * This is basically the inverse of calling {@link com.ctre.phoenix6.Utils#fpgaToCurrentTime(double) Utils.fpgaToCurrentTime()}.
     *
     * @param stateTimestampSeconds The SwerveDriveState timestamp in seconds
     * @return The equivalent NetworkTables timestamp in microseconds
     */
    public static long stateTimestampToNTTimestamp(double stateTimestampSeconds) {
        double NTTimestampSeconds = stateTimestampSeconds + (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds());
        return (long) (NTTimestampSeconds * 1000000.0);
    }
}
