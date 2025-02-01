package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingNearestPosition;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingPosition;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricVisualServoing;
import frc.robot.util.Aiming;
import java.util.function.DoubleSupplier;

public class AimingRoutines {
    private final CommandSwerveDrivetrain m_drivetrain;

    private final DoubleSupplier m_velocityXSupplier;
    private final DoubleSupplier m_velocityYSupplier;
    private final DoubleSupplier m_rotationalRateSupplier;
    private final DoubleSupplier m_deadbandSupplier;
    private final DoubleSupplier m_rotationalDeadbandSupplier;

    private final FieldCentric backupDrive;

    private final ProfiledFieldCentricFacingAngle driveFacingAngle = new ProfiledFieldCentricFacingAngle(
                    DriveConstants.ANGULAR_MOTION_CONSTRAINTS)
            .withPIDGains(DriveConstants.K_P, 0, DriveConstants.K_D)
            .withTolerance(DriveConstants.GOAL_TOLERANCE);

    private final ProfiledFieldCentricFacingPosition driveFacingPosition = new ProfiledFieldCentricFacingPosition(
                    DriveConstants.ANGULAR_MOTION_CONSTRAINTS)
            .withPIDGains(DriveConstants.K_P, 0, DriveConstants.K_D)
            .withTolerance(DriveConstants.GOAL_TOLERANCE);

    private final ProfiledFieldCentricFacingNearestPosition driveFacingNearestPosition =
            new ProfiledFieldCentricFacingNearestPosition(DriveConstants.ANGULAR_MOTION_CONSTRAINTS)
                    .withPIDGains(DriveConstants.K_P, 0, DriveConstants.K_D)
                    .withTolerance(DriveConstants.GOAL_TOLERANCE);

    private final ProfiledFieldCentricVisualServoing driveFacingVisionTarget = new ProfiledFieldCentricVisualServoing(
                    DriveConstants.ANGULAR_MOTION_CONSTRAINTS, VisionConstants.FRONT_LIMELIGHT_NAME)
            .withPIDGains(DriveConstants.K_P, 0, DriveConstants.K_D)
            .withTolerance(DriveConstants.GOAL_TOLERANCE);

    /* NetworkTables Logging */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    /**
     * Gets the ID of the primary in-view apriltag.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">limelight NetworkTables API</a>
     * @see {@link frc.robot.util.LimelightHelpers#getFiducialID(String) LimelightHelpers equivalent}
     */
    private final IntegerSubscriber apriltagID = inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME)
            .getIntegerTopic("tid")
            .subscribe(0);

    public AimingRoutines(
            CommandSwerveDrivetrain drivetrain,
            FieldCentric backupDrive,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            DoubleSupplier rotationalRateSupplier,
            DoubleSupplier deadbandSupplier,
            DoubleSupplier rotationalDeadbandSupplier) {
        m_drivetrain = drivetrain;
        this.backupDrive = backupDrive;
        m_velocityXSupplier = velocityXSupplier;
        m_velocityYSupplier = velocityYSupplier;
        m_rotationalRateSupplier = rotationalRateSupplier;
        m_deadbandSupplier = deadbandSupplier;
        m_rotationalDeadbandSupplier = rotationalDeadbandSupplier;
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
        return m_drivetrain
                // We have to reset the profile manually because the following request may or may not be a
                // profiled request
                .runOnce(driveFacingPosition::resetProfile)
                .andThen(
                        // Face either the blue or red reef
                        m_drivetrain
                                .applyRequest(() -> {
                                    Alliance alliance =
                                            DriverStation.getAlliance().orElse(null);
                                    if (alliance == Alliance.Blue) {
                                        driveFacingPosition.withTargetPosition(FieldConstants.BLUE_REEF_CENTER);
                                    } else if (alliance == Alliance.Red) {
                                        driveFacingPosition.withTargetPosition(FieldConstants.RED_REEF_CENTER);
                                    } else {
                                        DriverStation.reportWarning(
                                                "Driver Station not connected, robot will drive normally!", false);
                                        return backupDrive
                                                .withVelocityX(m_velocityXSupplier.getAsDouble())
                                                .withVelocityY(m_velocityYSupplier.getAsDouble())
                                                .withRotationalRate(m_rotationalRateSupplier.getAsDouble())
                                                .withDeadband(m_deadbandSupplier.getAsDouble())
                                                .withRotationalDeadband(m_rotationalDeadbandSupplier.getAsDouble());
                                    }
                                    return driveFacingPosition
                                            .withVelocityX(m_velocityXSupplier.getAsDouble())
                                            .withVelocityY(m_velocityYSupplier.getAsDouble())
                                            .withDeadband(m_deadbandSupplier.getAsDouble());
                                })
                                // This will return false if drive is applied instead of driveFacingPosition.
                                .until(driveFacingPosition::motionIsFinished))
                .andThen(m_drivetrain
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
                        .until(() -> Aiming.isReefTag((int) apriltagID.get())))
                // Drive facing perpendicular to the apriltag
                .andThen(m_drivetrain.applyProfiledRequest(() -> {
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
        return m_drivetrain
                // We have to reset the profile manually because the following request may or may not be a
                // profiled request
                .runOnce(driveFacingPosition::resetProfile)
                .andThen(m_drivetrain
                        .applyRequest(() -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(null);
                            if (alliance == Alliance.Blue) {
                                driveFacingPosition.withTargetPosition(FieldConstants.BLUE_REEF_CENTER);
                            } else if (alliance == Alliance.Red) {
                                driveFacingPosition.withTargetPosition(FieldConstants.RED_REEF_CENTER);
                            } else {
                                DriverStation.reportWarning(
                                        "Driver Station not connected, robot will drive normally!", false);
                                return backupDrive
                                        .withVelocityX(m_velocityXSupplier.getAsDouble())
                                        .withVelocityY(m_velocityYSupplier.getAsDouble())
                                        .withRotationalRate(m_rotationalRateSupplier.getAsDouble())
                                        .withDeadband(m_deadbandSupplier.getAsDouble())
                                        .withRotationalDeadband(m_rotationalDeadbandSupplier.getAsDouble());
                            }
                            return driveFacingPosition
                                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                                    .withDeadband(m_deadbandSupplier.getAsDouble());
                        })
                        // motionIsFinished() will return false if drive is applied instead of
                        // driveFacingPosition.
                        .until(() ->
                                driveFacingPosition.motionIsFinished() && Aiming.isReefTag((int) apriltagID.get())))
                // Face the selected tree
                .andThen(m_drivetrain.applyProfiledRequest(() -> driveFacingVisionTarget
                        .withVelocityX(m_velocityXSupplier.getAsDouble())
                        .withVelocityY(m_velocityYSupplier.getAsDouble())
                        .withDeadband(m_deadbandSupplier.getAsDouble())));
    }
    ;

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
        return m_drivetrain
                // We have to reset the profile manually because the following request may or may not be a
                // profiled request
                .runOnce(driveFacingNearestPosition::resetProfile)
                .andThen(m_drivetrain
                        .applyRequest(() -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(null);
                            if (alliance == Alliance.Blue) {
                                driveFacingNearestPosition.withTargetPositions(FieldConstants.BLUE_REEF_TREE_POSITIONS);
                            } else if (alliance == Alliance.Red) {
                                driveFacingNearestPosition.withTargetPositions(FieldConstants.RED_REEF_TREE_POSITIONS);
                            } else {
                                DriverStation.reportWarning(
                                        "Driver Station not connected, robot will drive normally!", false);
                                return backupDrive
                                        .withVelocityX(m_velocityXSupplier.getAsDouble())
                                        .withVelocityY(m_velocityYSupplier.getAsDouble())
                                        .withRotationalRate(m_rotationalRateSupplier.getAsDouble())
                                        .withDeadband(m_deadbandSupplier.getAsDouble())
                                        .withRotationalDeadband(m_rotationalDeadbandSupplier.getAsDouble());
                            }
                            return driveFacingNearestPosition
                                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                                    .withDeadband(m_deadbandSupplier.getAsDouble());
                        })
                        .until(() ->
                                driveFacingPosition.motionIsFinished() && Aiming.isReefTag((int) apriltagID.get())))
                // Face the tag
                .andThen(m_drivetrain.applyProfiledRequest(() -> driveFacingVisionTarget
                        .withVelocityX(m_velocityXSupplier.getAsDouble())
                        .withVelocityY(m_velocityYSupplier.getAsDouble())
                        .withDeadband(m_deadbandSupplier.getAsDouble())));
    }
}
