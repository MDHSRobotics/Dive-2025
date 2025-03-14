package frc.robot.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class DriveTelemetry {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    /**
     * Provides the robot orientation to the front limelight for <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2">megatag2</a>.
     * The LimelightHelpers equivalent to this is {@link frc.robot.util.LimelightHelpers#SetRobotOrientation(String, double, double, double, double, double, double) SetRobotOrientation()}.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">NetworkTables API documentation</a>
     */
    private final DoubleArrayPublisher megatag2FrontUpdater = inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish();

    private final DoubleArrayPublisher megatag2BackUpdater = inst.getTable(VisionConstants.BACK_LIMELIGHT_NAME)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish();

    /** Limelight requires this to be a array of size 6. */
    private double[] megatag2Orientation = new double[6];

    /** Used to derive linear accel */
    private double previousLinearSpeed = 0;
    /** Used to derive angular accel */
    private double previousYawRate = 0;

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePosePub = driveStateTable
            .getSubTable("Poses")
            .getStructTopic("Pose", Pose2d.struct)
            .publish();
    private final StructPublisher<ChassisSpeeds> driveSpeedsPub =
            driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStatesPub = driveStateTable
            .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
            .publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargetsPub = driveStateTable
            .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
            .publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositionsPub = driveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
            .publish();
    private final DoublePublisher driveOdometryFrequencyPub =
            driveStateTable.getDoubleTopic("OdometryFrequency").publish();
    private final DoublePublisher driveOdometryPeriodPub =
            driveStateTable.getDoubleTopic("OdometryPeriod").publish();
    private final DoublePublisher linearSpeedPub =
            driveStateTable.getDoubleTopic("Linear Speed").publish();
    private final DoublePublisher linearAccelPub =
            driveStateTable.getDoubleTopic("Linear Accel").publish();
    private final DoublePublisher angularAccelPub =
            driveStateTable.getDoubleTopic("Angular Accel").publish();

    private final double[] poseArray = new double[3];
    private final double[] moduleStatesArray = new double[8];
    private final double[] moduleTargetsArray = new double[8];

    /** Accept the swerve drive state and log it to NetworkTables and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        long timestamp = stateTimestampToNTTimestamp(state.Timestamp);

        /* Send the robot orientation to the limelight for megatag2 */
        megatag2Orientation[0] = state.Pose.getRotation().getDegrees();
        megatag2Orientation[1] = state.Speeds.omegaRadiansPerSecond * 180.0 / Math.PI;
        megatag2FrontUpdater.set(megatag2Orientation, timestamp);
        megatag2BackUpdater.set(megatag2Orientation, timestamp);
        inst.flush();

        /* Telemeterize the swerve drive state */
        drivePosePub.set(state.Pose, timestamp);
        driveSpeedsPub.set(state.Speeds, timestamp);
        driveModuleStatesPub.set(state.ModuleStates, timestamp);
        driveModuleTargetsPub.set(state.ModuleTargets, timestamp);
        driveModulePositionsPub.set(state.ModulePositions, timestamp);
        driveOdometryFrequencyPub.set(1.0 / state.OdometryPeriod, timestamp);
        driveOdometryPeriodPub.set(state.OdometryPeriod, timestamp);

        double linearSpeed = Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);
        linearSpeedPub.set(linearSpeed, timestamp);

        linearAccelPub.set((linearSpeed - previousLinearSpeed) / state.OdometryPeriod, timestamp);
        // Update previous linear velocity
        previousLinearSpeed = linearSpeed;

        angularAccelPub.set((state.Speeds.omegaRadiansPerSecond - previousYawRate) / state.OdometryPeriod, timestamp);
        // Update previous angular velocity
        previousYawRate = state.Speeds.omegaRadiansPerSecond;

        /* Also write to log file */
        poseArray[0] = state.Pose.getX();
        poseArray[1] = state.Pose.getY();
        poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
            moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
            moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
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
