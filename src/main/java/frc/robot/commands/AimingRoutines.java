package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.requests.DriveFacingAngle;
import frc.robot.subsystems.drive.requests.DriveFacingNearestPosition;
import frc.robot.subsystems.drive.requests.DriveFacingPosition;
import frc.robot.subsystems.drive.requests.DriveWithVisualServoing;
import frc.robot.subsystems.drive.requests.ProfiledXYHeadingAlignment;
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

    private final NetworkTable loggingTable = inst.getTable("Swerve Requests");

    private final DriveFacingAngle driveFacingAngle = new DriveFacingAngle(
                    K_ANGULAR_P, 0.0, 0.0, MAX_ANGULAR_RATE, loggingTable)
            .withTolerance(GOAL_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final DriveFacingPosition driveFacingPosition = new DriveFacingPosition(
                    K_ANGULAR_P, 0.0, 0.0, MAX_ANGULAR_RATE, loggingTable)
            .withTolerance(GOAL_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final DriveFacingNearestPosition driveFacingNearestPosition = new DriveFacingNearestPosition(
                    K_ANGULAR_P, 0.0, 0.0, MAX_ANGULAR_RATE, loggingTable)
            .withTolerance(GOAL_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final DriveWithVisualServoing driveFacingVisionTarget = new DriveWithVisualServoing(
                    K_ANGULAR_P, 0.0, 0.0, MAX_ANGULAR_RATE, cameraTable, loggingTable)
            .withTolerance(GOAL_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final ProfiledXYHeadingAlignment driveToPosition = new ProfiledXYHeadingAlignment(
                    K_ANGULAR_P, 0.0, 0.0, MAX_ANGULAR_RATE, LINEAR_MOTION_CONSTRAINTS, loggingTable)
            .withTranslationalPIDGains(K_TRANSLATION_P, 0, 0)
            .withTolerance(GOAL_TOLERANCE)
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

    public Command alignWithStation(boolean leftStation) {
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

    public Command alignWithCage() {
        return m_drivetrain.applyResettableRequest(() -> {
            Alliance alliance = DriverStation.getAlliance().orElseThrow();
            if (alliance == Alliance.Blue) {
                driveFacingAngle.withTargetDirection(Rotation2d.kCW_90deg);
            } else {
                driveFacingAngle.withTargetDirection(Rotation2d.kCCW_90deg);
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
                m_drivetrain
                        .applyResettableRequest(() -> {
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
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
                                    driveFacingAngle.resetProfile();
                                    m_drivetrain.setControl(driveFacingAngle
                                            .withVelocityX(m_velocityXSupplier.getAsDouble())
                                            .withVelocityY(m_velocityYSupplier.getAsDouble())
                                            .withTargetDirection(Aiming.nearestRotation(
                                                    m_drivetrain.getState().Pose.getRotation(),
                                                    FieldConstants.REEF_WALL_ROTATIONS))
                                            .withDeadband(m_deadbandSupplier.getAsDouble()));
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
                }));
    }

    /*
     * This lengthy sequence is for locking on to a tree. Here is the explanation:
     * Once the driver presses this button, the robot will rotate to face the center of the reef.
     * The point of this is to give the camera a chance to see the correct tag without requiring the driver to rotate manually.
     * If a reef tag is in view or enters view when the movement is finished, the robot will start aiming at an offset from the tag using tx.
     * The offset is to the left if the operator has selected the left tree,
     * or to the right if the operator has selected the right tree.
     * (They can change their selection any time.)
     * If the robot loses sight of all reef tags, it will finish rotating based on the last known tx value,
     * and then stop rotating unless a tag returns in view.
     * This is to provide the driver a chance to drive forwards/backwards/left/right to align the robot to a branch themself.
     */
    public Command orientToFaceTree() {
        return Commands.sequence(
                m_drivetrain
                        .applyResettableRequest(() -> {
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
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
                        .until(() ->
                                driveFacingPosition.motionIsFinished() && Aiming.isReefTag((int) apriltagID.get())),
                // Face the selected tree
                m_drivetrain.applyResettableRequest(() -> driveFacingVisionTarget
                        .withVelocityX(m_velocityXSupplier.getAsDouble())
                        .withVelocityY(m_velocityYSupplier.getAsDouble())
                        .withDeadband(m_deadbandSupplier.getAsDouble())));
    }

    /**
     * This is a backup sequence for locking onto the tree without needing to see apriltags.
     * Once the driver presses this button, the robot will rotate to face the nearest tree.
     * If a reef tag is in view or enters view when the movement is finished, the robot will return to limelight aiming.
     * It will start aiming at an offset from the tag using tx.
     * The offset is to the left if the operator has selected the left tree,
     * or to the right if the operator has selected the right tree.
     * (They can change their selection any time.)
     * If the robot loses sight of all reef tags, it will finish rotating based on the last known tx value,
     * and then stop rotating unless a tag returns in view.
     * This is to provide the driver a chance to drive forwards/backwards/left/right to align the robot to a branch themself.
     */
    public Command orientToFaceTreeWithoutLimelight() {
        return Commands.sequence(
                m_drivetrain
                        .applyResettableRequest(() -> {
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
                            if (alliance == Alliance.Blue) {
                                driveFacingNearestPosition.withTargetPositions(FieldConstants.BLUE_REEF_TREE_POSITIONS);
                            } else if (alliance == Alliance.Red) {
                                driveFacingNearestPosition.withTargetPositions(FieldConstants.RED_REEF_TREE_POSITIONS);
                            }
                            return driveFacingNearestPosition
                                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                                    .withDeadband(m_deadbandSupplier.getAsDouble());
                        })
                        .until(() ->
                                driveFacingPosition.motionIsFinished() && Aiming.isReefTag((int) apriltagID.get())),
                // Face the tag
                m_drivetrain.applyResettableRequest(() -> driveFacingVisionTarget
                        .withVelocityX(m_velocityXSupplier.getAsDouble())
                        .withVelocityY(m_velocityYSupplier.getAsDouble())
                        .withDeadband(m_deadbandSupplier.getAsDouble())));
    }

    public Command driveToTree() {
        return driveToNearestPositionCommand(
                FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS, FieldConstants.RED_REEF_TREE_AIMING_POSITIONS);
    }

    /**
     * This command attempts to drive in front of the cage,
     * and rotate so that the robot's side opening is facing the cage.
     */
    public Command driveInFrontOfCage() {
        return driveToNearestPositionCommand(
                FieldConstants.BLUE_CAGE_STARTING_POSITIONS, FieldConstants.RED_CAGE_STARTING_POSITIONS);
    }

    /**
     * This command attempts to automatically drive into the cage.
     */
    public Command driveIntoCage() {
        return driveToNearestPositionCommand(FieldConstants.BLUE_CAGE_POSITIONS, FieldConstants.RED_CAGE_POSITIONS);
    }

    /**
     * Drives to the nearest position in the list of the current alliance.
     * @param bluePositions The list of blue positions to select from
     * @param redPositions The list of red positions to select from
     * @return A deferred command that will not generate the path until it is scheduled.
     */
    // private Command driveToNearestPositionCommand(List<Pose2d> bluePositions, List<Pose2d> redPositions) {
    //     return m_drivetrain.defer(() -> {
    //         Pose2d currentPose = m_drivetrain.getState().Pose;
    //         Alliance alliance = DriverStation.getAlliance().orElseThrow();
    //         Pose2d treePose;
    //         if (alliance == Alliance.Blue) {
    //             treePose = currentPose.nearest(bluePositions);
    //         } else {
    //             treePose = currentPose.nearest(redPositions);
    //         }
    //         Translation2d currentToTargetDistance = treePose.getTranslation().minus(currentPose.getTranslation());
    //         Rotation2d currentToTargetRotation = currentToTargetDistance.getAngle();

    //         List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //                 new Pose2d(currentPose.getX(), currentPose.getY(), currentToTargetRotation),
    //                 new Pose2d(treePose.getX(), treePose.getY(), currentToTargetRotation));

    //         PathPlannerPath path = new PathPlannerPath(
    //                 waypoints,
    //                 DriveConstants.PATHFINDING_CONSTRAINTS,
    //                 null,
    //                 new GoalEndState(0, treePose.getRotation()));
    //         path.preventFlipping = true;

    //         // System.out.println("Target x: " + Units.metersToInches(treePose.getX()) + "\nTarget y: "
    //         //         + Units.metersToInches(treePose.getY()) + "\nTarget direction: "
    //         //         + treePose.getRotation().getDegrees());

    //         return AutoBuilder.followPath(path);
    //     });
    // }
    private Command driveToNearestPositionCommand(List<Pose2d> bluePositions, List<Pose2d> redPositions) {
        return m_drivetrain.startRun(
                () -> {
                    driveToPosition.resetProfile();
                    Pose2d currentPose = m_drivetrain.getState().Pose;
                    Alliance alliance = DriverStation.getAlliance().orElseThrow();
                    Pose2d treePose;
                    if (alliance == Alliance.Blue) {
                        treePose = currentPose.nearest(bluePositions);
                    } else {
                        treePose = currentPose.nearest(redPositions);
                    }
                    m_drivetrain.setControl(driveToPosition.withTargetPose(treePose));
                },
                () -> m_drivetrain.setControl(driveToPosition));
    }
}
