package frc.robot.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;
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
import frc.robot.Constants.VisionConstants;

public class DriveTelemetry {
    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /**
     * Provides the robot orientation to the limelight for <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2">megatag2</a>.
     * The LimelightHelpers equivalent to this is {@link frc.robot.util.LimelightHelpers#SetRobotOrientation(String, double, double, double, double, double, double) SetRobotOrientation()}.
     */
    private final DoubleArrayPublisher megatag2Updater = inst.getTable(VisionConstants.LIMELIGHT_NAME)
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
    private final DoublePublisher driveTimestamp =
            driveStateTable.getDoubleTopic("Timestamp").publish();
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
        /* Send the robot orientation to the limelight for megatag2 */
        megatag2Orientation[0] = state.Pose.getRotation().getDegrees();
        megatag2Orientation[1] = state.Speeds.omegaRadiansPerSecond * 180.0 / Math.PI;
        megatag2Updater.set(megatag2Orientation);
        // Send the value to the limelight immediately, instead of waiting for the next cycle. This doesn't matter as
        // much for the other publishers.
        inst.flush();

        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
        driveOdometryPeriod.set(state.OdometryPeriod);
        angularAccel.set((state.Speeds.omegaRadiansPerSecond - previousYawRate) / state.OdometryPeriod);

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
}
