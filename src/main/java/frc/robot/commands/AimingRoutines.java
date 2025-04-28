package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.requests.DriveFacingAngle;
import frc.robot.subsystems.drive.requests.DriveFacingPosition;
import frc.robot.subsystems.drive.requests.DriveToPose;
import frc.robot.util.Aiming;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * This class provides instanced command factories for swerve drive aiming.
 * <p>
 * Since these commands are so long, they have been moved here to shorten RobotContainer.
 */
public class AimingRoutines {
    private final CommandSwerveDrivetrain m_drivetrain;

    private final DoubleSupplier m_velocityXSupplier;
    private final DoubleSupplier m_velocityYSupplier;
    private final DoubleSupplier m_deadbandSupplier;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable cameraTable = inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME);

    private final DriveFacingAngle driveFacingAngle = new DriveFacingAngle(ROTATION_PID.kP, MAX_ANGULAR_RATE)
            .withTolerance(HEADING_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final DriveFacingPosition driveFacingPosition = new DriveFacingPosition(ROTATION_PID.kP, MAX_ANGULAR_RATE)
            .withTolerance(HEADING_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final DriveToPose driveToPose = new DriveToPose(
                    TRANSLATION_PID.kP,
                    ROTATION_PID.kP,
                    MAX_ANGULAR_RATE,
                    LINEAR_MOTION_CONSTRAINTS,
                    PATHPLANNER_CONFIG,
                    MAX_STEER_VELOCITY,
                    Constants.UPDATE_PERIOD)
            .withHeadingTolerance(HEADING_TOLERANCE)
            .withLinearTolerance(LINEAR_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    /**
     * Gets the ID of the primary in-view apriltag.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">limelight NetworkTables API</a>
     * @see {@link frc.robot.util.LimelightHelpers#getFiducialID(String) LimelightHelpers equivalent}
     */
    private final IntegerSubscriber apriltagID =
            cameraTable.getIntegerTopic("tid").subscribe(0);

    /**
     * Constructs an object that provides <a href="https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories">instanced command factories</a> for swerve drive aiming.
     * @param drivetrain The drivetrain to drive and aim with.
     * @param velocityXSupplier A method reference or lambda that returns X velocity.
     * @param velocityYSupplierA method reference or lambda that returns Y velocity.
     * @param deadbandSupplier A method reference or lambda that returns deadband.
     */
    public AimingRoutines(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            DoubleSupplier deadbandSupplier) {
        m_drivetrain = drivetrain;
        m_velocityXSupplier = velocityXSupplier;
        m_velocityYSupplier = velocityYSupplier;
        m_deadbandSupplier = deadbandSupplier;
    }

    public Command alignWithCoralStation(boolean leftStation) {
        return m_drivetrain.applyResettableRequest(() -> {
            Alliance alliance = DriverStation.getAlliance().orElseThrow();
            if (leftStation) {
                if (alliance == Alliance.Blue) {
                    driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[13]);
                } else if (alliance == Alliance.Red) {
                    driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[1]);
                }
            } else {
                if (alliance == Alliance.Blue) {
                    driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[12]);
                } else if (alliance == Alliance.Red) {
                    driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[2]);
                }
            }

            return driveFacingAngle
                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                    .withDeadband(m_deadbandSupplier.getAsDouble());
        });
    }

    public Command alignWithProcessor() {
        return m_drivetrain.applyResettableRequest(() -> {
            Alliance alliance = DriverStation.getAlliance().orElseThrow();
            if (alliance == Alliance.Blue) {
                driveFacingAngle.withTargetDirection(Rotation2d.kCCW_90deg);
            } else {
                driveFacingAngle.withTargetDirection(Rotation2d.kCW_90deg);
            }

            return driveFacingAngle
                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                    .withDeadband(m_deadbandSupplier.getAsDouble());
        });
    }

    /*
     * This lengthy sequence is for locking on to a reef wall. Here is the explanation:
     * Once the driver presses this button, the robot will orient to face the center of the reef.
     * The point of this is to give the camera a chance to see the correct tag without requiring the driver to rotate manually.
     * Once this movement is finished, the robot will snap to the closest angle that orients itself with a reef wall.
     * If a reef tag is in view or enters view, the robot will instead orient itself perpendicular to the tag.
     * It does this by looking for the closest tag (based on tag size in the camera view),
     * and snapping to an angle that faces the wall (based on the tag's id).
     * The point of this is to allow the robot to snap to each wall as it rotates around the reef.
     * If the robot loses sight of all reef tags, it will finish rotating based on the last known angle,
     * and then stop rotating unless a tag returns in view.
     */
    public Command orientToFaceReefWall() {
        return Commands.sequence(
                        m_drivetrain.runOnce(() -> m_drivetrain.updateVisionTarget(true)),
                        m_drivetrain
                                .applyResettableRequest(() -> {
                                    Alliance alliance =
                                            DriverStation.getAlliance().orElseThrow();
                                    if (alliance == Alliance.Blue) {
                                        driveFacingPosition.withTargetPosition(FieldConstants.BLUE_REEF_CENTER);
                                    } else if (alliance == Alliance.Red) {
                                        driveFacingPosition.withTargetPosition(FieldConstants.RED_REEF_CENTER);
                                    }
                                    return driveFacingPosition
                                            .withVelocityX(m_velocityXSupplier.getAsDouble())
                                            .withVelocityY(m_velocityYSupplier.getAsDouble())
                                            .withDeadband(m_deadbandSupplier.getAsDouble());
                                })
                                .until(driveFacingPosition::motionIsFinished),
                        m_drivetrain
                                .startRun(
                                        () -> {
                                            driveFacingAngle.resetRequest();
                                            driveFacingAngle.withTargetDirection(Aiming.nearestRotation(
                                                    m_drivetrain.getState().Pose.getRotation(),
                                                    FieldConstants.REEF_WALL_ROTATIONS));
                                        },
                                        () -> m_drivetrain.setControl(driveFacingAngle
                                                .withVelocityX(m_velocityXSupplier.getAsDouble())
                                                .withVelocityY(m_velocityYSupplier.getAsDouble())
                                                .withDeadband(m_deadbandSupplier.getAsDouble())))
                                .until(() -> Aiming.isReefTag((int) apriltagID.get())),
                        m_drivetrain.applyResettableRequest(() -> {
                            int id = (int) apriltagID.get();
                            if (Aiming.isReefTag(id)) {
                                driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[id]);
                            }
                            return driveFacingAngle
                                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                                    .withDeadband(m_deadbandSupplier.getAsDouble());
                        }))
                .finallyDo(() -> m_drivetrain.updateVisionTarget(false));
    }

    /**
     * This command will generate a path to the nearest tree and follow it.
     * Once it finishes following the path, it will attempt to correct any inaccuracies in its position until interrupted.
     */
    public Command driveToTree() {
        return Commands.sequence(
                        m_drivetrain.defer(() -> {
                            m_drivetrain.updateVisionTarget(true);
                            SwerveDriveState currentState = m_drivetrain.getState();
                            Pose2d currentPose = m_drivetrain.getState().Pose;
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
                            Pose2d treePose;
                            if (alliance == Alliance.Blue) {
                                treePose = currentPose.nearest(FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS);
                            } else {
                                treePose = currentPose.nearest(FieldConstants.RED_REEF_TREE_AIMING_POSITIONS);
                            }
                            return generatePath(currentState, treePose, ON_THE_FLY_CONSTRAINTS, false);
                        }),
                        m_drivetrain.startRun(
                                () -> {
                                    driveToPose.resetRequest();
                                    Pose2d currentPose = m_drivetrain.getState().Pose;
                                    Alliance alliance =
                                            DriverStation.getAlliance().orElseThrow();
                                    Pose2d treePose;
                                    if (alliance == Alliance.Blue) {
                                        treePose = currentPose.nearest(FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS);
                                    } else {
                                        treePose = currentPose.nearest(FieldConstants.RED_REEF_TREE_AIMING_POSITIONS);
                                    }
                                    driveToPose.withTargetPose(treePose);
                                },
                                () -> m_drivetrain.setControl(driveToPose)))
                .finallyDo(() -> m_drivetrain.updateVisionTarget(false));
    }

    public Command driveToTreeSimple() {
        return m_drivetrain
                .startRun(
                        () -> {
                            m_drivetrain.updateVisionTarget(true);
                            driveToPose.resetRequest();
                            Pose2d currentPose = m_drivetrain.getState().Pose;
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
                            Pose2d treePose;
                            if (alliance == Alliance.Blue) {
                                treePose = currentPose.nearest(FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS);
                            } else {
                                treePose = currentPose.nearest(FieldConstants.RED_REEF_TREE_AIMING_POSITIONS);
                            }
                            driveToPose.withTargetPose(treePose);
                        },
                        () -> m_drivetrain.setControl(driveToPose))
                .finallyDo(() -> m_drivetrain.updateVisionTarget(false));
    }

    public Command driveToCoralStation() {
        return Commands.sequence(
                m_drivetrain.defer(() -> {
                    SwerveDriveState currentState = m_drivetrain.getState();
                    Pose2d currentPose = m_drivetrain.getState().Pose;
                    Alliance alliance = DriverStation.getAlliance().orElseThrow();
                    Pose2d coralStationPose;
                    if (alliance == Alliance.Blue) {
                        coralStationPose = currentPose.nearest(FieldConstants.BLUE_CORAL_STATION_POSES);
                    } else {
                        coralStationPose = currentPose.nearest(FieldConstants.RED_CORAL_STATION_POSES);
                    }
                    return generatePath(currentState, coralStationPose, CORAL_STATION_CONSTRAINTS, false);
                }),
                m_drivetrain.startRun(
                        () -> {
                            driveToPose.resetRequest();
                            Pose2d currentPose = m_drivetrain.getState().Pose;
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
                            Pose2d coralStationPose;
                            if (alliance == Alliance.Blue) {
                                coralStationPose = currentPose.nearest(FieldConstants.BLUE_CORAL_STATION_POSES);
                            } else {
                                coralStationPose = currentPose.nearest(FieldConstants.RED_CORAL_STATION_POSES);
                            }
                            driveToPose.withTargetPose(coralStationPose);
                        },
                        () -> m_drivetrain.setControl(driveToPose)));
    }

    /**
     * Generates a path-following command that drives the robot to a target position and rotation.
     * <p>
     * The driver must make sure to NEVER be driving straight away from the target when this method is called,
     * or else it will generate a path that does not respect the robot's momentum.
     * See "gifs/Backwards_Curve_With_Initial_Velocity.gif" for a visualization of this situation.
     *
     * @param currentState The current pose and speeds of the robot
     * @param targetPose The target position and rotation of the robot
     * @param pathConstraints The constraints to use while driving
     * @param allowTargetFlipping Whether to flip targets when the alliance is red
     * @return The path following command
     */
    private static Command generatePath(
            SwerveDriveState currentState,
            Pose2d targetPose,
            PathConstraints pathConstraints,
            boolean allowTargetFlipping) {
        // Save the target position and rotation for potential flipping
        Translation2d targetPosition = targetPose.getTranslation();
        Rotation2d targetRotation = targetPose.getRotation();

        if (allowTargetFlipping && DriverStation.getAlliance().orElseThrow().equals(Alliance.Red)) {
            targetPosition = FlippingUtil.flipFieldPosition(targetPosition);
            targetRotation = FlippingUtil.flipFieldRotation(targetRotation);
        }

        // In simulation, the robot has a chance of being perfectly still, which would require us to use the direction
        // to the target instead.
        Rotation2d directionOfTravel;
        if (currentState.Speeds.vxMetersPerSecond == 0.0 && currentState.Speeds.vyMetersPerSecond == 0.0) {
            Translation2d directionToTarget = targetPosition.minus(currentState.Pose.getTranslation());
            directionOfTravel = new Rotation2d(directionToTarget.getX(), directionToTarget.getY());
        } else {
            // You need to convert the robot-relative speeds to field-relative speeds to find the actual direction of
            // travel.
            ChassisSpeeds currentFieldSpeeds =
                    ChassisSpeeds.fromRobotRelativeSpeeds(currentState.Speeds, currentState.Pose.getRotation());
            directionOfTravel =
                    new Rotation2d(currentFieldSpeeds.vxMetersPerSecond, currentFieldSpeeds.vyMetersPerSecond);
        }

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel for the first waypoint, and the end
        // robot rotation for the last waypoint.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentState.Pose.getX(), currentState.Pose.getY(), directionOfTravel),
                new Pose2d(targetPosition.getX(), targetPosition.getY(), targetRotation));

        // DO NOT enter an ideal starting state for on-the-fly paths.
        // PathPlanner should generate the trajectory when AutoBuilder creates the command.
        // AutoBuilder will do a much better job because it has more information about the robot.
        PathPlannerPath path =
                new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(0, targetRotation));
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
}
