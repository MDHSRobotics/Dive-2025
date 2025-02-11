package frc.robot.commands;

import static frc.robot.Constants.K_DT;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.requests.ProfiledDriveFacingAngle;
import frc.robot.subsystems.drive.requests.ProfiledDriveFacingNearestPosition;
import frc.robot.subsystems.drive.requests.ProfiledDriveFacingPosition;
import frc.robot.subsystems.drive.requests.ProfiledDriveWithVisualServoing;
import frc.robot.subsystems.drive.requests.ProfiledXYHeadingAlignment;
import frc.robot.util.Aiming;
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

    private final ProfiledDriveFacingAngle driveFacingAngle = new ProfiledDriveFacingAngle(
                    ANGULAR_MOTION_CONSTRAINTS, K_DT)
            .withPIDGains(K_ANGULAR_P, 0, K_ANGULAR_D)
            .withTolerance(GOAL_TOLERANCE);

    private final ProfiledDriveFacingPosition driveFacingPosition = new ProfiledDriveFacingPosition(
                    ANGULAR_MOTION_CONSTRAINTS, K_DT)
            .withPIDGains(K_ANGULAR_P, 0, K_ANGULAR_D)
            .withTolerance(GOAL_TOLERANCE);

    private final ProfiledDriveFacingNearestPosition driveFacingNearestPosition =
            new ProfiledDriveFacingNearestPosition(ANGULAR_MOTION_CONSTRAINTS, K_DT)
                    .withPIDGains(K_ANGULAR_P, 0, K_ANGULAR_D)
                    .withTolerance(GOAL_TOLERANCE);

    private final ProfiledDriveWithVisualServoing driveFacingVisionTarget = new ProfiledDriveWithVisualServoing(
                    ANGULAR_MOTION_CONSTRAINTS, K_DT, VisionConstants.FRONT_LIMELIGHT_NAME)
            .withPIDGains(K_ANGULAR_P, 0, K_ANGULAR_D)
            .withTolerance(GOAL_TOLERANCE);

    private final ProfiledXYHeadingAlignment driveToPosition = new ProfiledXYHeadingAlignment(
                    LINEAR_MOTION_CONSTRAINTS, ANGULAR_MOTION_CONSTRAINTS, K_DT)
            .withTranslationalPIDGains(K_TRANSLATION_P, 0, K_TRANSLATION_D)
            .withRotationalPIDGains(K_ANGULAR_P, 0, K_ANGULAR_D);

    /**
     * Gets the ID of the primary in-view apriltag.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">limelight NetworkTables API</a>
     * @see {@link frc.robot.util.LimelightHelpers#getFiducialID(String) LimelightHelpers equivalent}
     */
    private final IntegerSubscriber apriltagID = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.FRONT_LIMELIGHT_NAME)
            .getIntegerTopic("tid")
            .subscribe(0);

    /**
     * Constructs an object that provides <a href="https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories">instanced command factories</a> for swerve drive aiming.
     * @param drivetrain The drivetrain to drive and aim with.
     * @param backupDrive A backup driving request to use in case the custom requests cannot be used.
     * @param velocityXSupplier A method reference or lambda that returns X velocity.
     * @param velocityYSupplierA method reference or lambda that returns Y velocity.
     * @param rotationalRateSupplier A method reference or lambda that returns rotational rate. Only used in backup driving.
     * @param deadbandSupplier A method reference or lambda that returns deadband.
     * @param rotationalDeadbandSupplier A method reference or lambda that returns rotational deadband. Only used in backup driving.
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
                        .applyProfiledRequest(() -> {
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
                m_drivetrain.applyProfiledRequest(() -> {
                    int id = (int) apriltagID.get();
                    if (Aiming.isReefTag(id)) {
                        driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[id - 1]);
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
                        .applyProfiledRequest(() -> {
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
                m_drivetrain.applyProfiledRequest(() -> driveFacingVisionTarget
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
                        .applyProfiledRequest(() -> {
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
                m_drivetrain.applyProfiledRequest(() -> driveFacingVisionTarget
                        .withVelocityX(m_velocityXSupplier.getAsDouble())
                        .withVelocityY(m_velocityYSupplier.getAsDouble())
                        .withDeadband(m_deadbandSupplier.getAsDouble())));
    }

    /**
     * This command attempts to drive in front of the cage,
     * and rotate so that the robot's side opening is facing the cage.
     */
    public Command driveInFrontOfCage() {
        return m_drivetrain.startRun(
                () -> {
                    driveToPosition.resetProfile();
                    Alliance alliance = DriverStation.getAlliance().orElseThrow();
                    Pose2d currentPose = m_drivetrain.getState().Pose;
                    if (alliance == Alliance.Blue) {
                        driveToPosition.withTargetPose(
                                currentPose.nearest(FieldConstants.BLUE_CAGE_STARTING_POSITIONS));
                    } else {
                        driveToPosition.withTargetPose(currentPose.nearest(FieldConstants.RED_CAGE_STARTING_POSITIONS));
                    }
                    m_drivetrain.setControl(driveToPosition);
                },
                () -> m_drivetrain.setControl(driveToPosition));
    }

    /**
     * This command attempts to automatically drive into the cage.
     */
    public Command driveIntoCage() {
        return m_drivetrain.startRun(
                () -> {
                    driveToPosition.resetProfile();
                    Alliance alliance = DriverStation.getAlliance().orElseThrow();
                    Pose2d currentPose = m_drivetrain.getState().Pose;
                    if (alliance == Alliance.Blue) {
                        driveToPosition.withTargetPose(currentPose.nearest(FieldConstants.BLUE_CAGE_POSITIONS));
                    } else {
                        driveToPosition.withTargetPose(currentPose.nearest(FieldConstants.RED_CAGE_POSITIONS));
                    }
                    m_drivetrain.setControl(driveToPosition);
                },
                () -> m_drivetrain.setControl(driveToPosition));
    }
}
