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
    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /**
     * Provides the robot orientation to the front limelight for <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2">megatag2</a>.
     * The LimelightHelpers equivalent to this is {@link frc.robot.util.LimelightHelpers#SetRobotOrientation(String, double, double, double, double, double, double) SetRobotOrientation()}.
     */
    private final DoubleArrayPublisher megatag2FrontUpdater = inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish();
    /**
     * Provides the robot orientation to the back limelight for <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2">megatag2</a>.
     * The LimelightHelpers equivalent to this is {@link frc.robot.util.LimelightHelpers#SetRobotOrientation(String, double, double, double, double, double, double) SetRobotOrientation()}.
     */
    private final DoubleArrayPublisher megatag2BackUpdater = inst.getTable(VisionConstants.BACK_LIMELIGHT_NAME)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish();

    private double[] megatag2Orientation = new double[6];

    /** Used to derive angular accel */
    private double previousYawRate = 0;

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose =
            driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds =
            driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
            .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
            .publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
            .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
            .publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
            .publish();
    private final DoublePublisher driveOdometryFrequency =
            driveStateTable.getDoubleTopic("OdometryFrequency").publish();
    private final DoublePublisher driveOdometryPeriod =
            driveStateTable.getDoubleTopic("OdometryPeriod").publish();
    private final DoublePublisher angularAccel =
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
        drivePose.set(state.Pose, timestamp);
        driveSpeeds.set(state.Speeds, timestamp);
        driveModuleStates.set(state.ModuleStates, timestamp);
        driveModuleTargets.set(state.ModuleTargets, timestamp);
        driveModulePositions.set(state.ModulePositions, timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod, timestamp);
        driveOdometryPeriod.set(state.OdometryPeriod, timestamp);
        angularAccel.set((state.Speeds.omegaRadiansPerSecond - previousYawRate) / state.OdometryPeriod, timestamp);

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
    private static long stateTimestampToNTTimestamp(double stateTimestampSeconds) {
        double NTTimestampSeconds = stateTimestampSeconds + (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds());
        return (long) (NTTimestampSeconds * 1000000.0);
    }
}
