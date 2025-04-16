package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.RobotContainer.CageLocation;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.requests.DriveFacingAngle;
import frc.robot.subsystems.drive.requests.DriveFacingPosition;
import frc.robot.subsystems.drive.requests.DriveWithVisualServoing;
import frc.robot.subsystems.drive.requests.XYHeadingAlignment;
import frc.robot.util.Aiming;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

    private final Supplier<CageLocation> m_cageSupplier;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable cameraTable = inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME);

    private final NetworkTable loggingTable = inst.getTable("Swerve Requests");

    private final DriveFacingAngle driveFacingAngle = new DriveFacingAngle(
                    ROTATION_PID.kP, MAX_ANGULAR_RATE, loggingTable)
            .withTolerance(HEADING_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final DriveFacingPosition driveFacingPosition = new DriveFacingPosition(
                    ROTATION_PID.kP, MAX_ANGULAR_RATE, loggingTable)
            .withTolerance(HEADING_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final DriveWithVisualServoing driveFacingVisionTarget = new DriveWithVisualServoing(
                    ROTATION_PID.kP, MAX_ANGULAR_RATE, cameraTable, loggingTable)
            .withTolerance(HEADING_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final XYHeadingAlignment driveToPosition = new XYHeadingAlignment(
                    TRANSLATION_PID.kP,
                    ROTATION_PID.kP,
                    MAX_ANGULAR_RATE,
                    LINEAR_MOTION_CONSTRAINTS,
                    PATHPLANNER_CONFIG,
                    MAX_STEER_VELOCITY,
                    Constants.UPDATE_PERIOD,
                    loggingTable)
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
            DoubleSupplier deadbandSupplier,
            Supplier<CageLocation> cageSupplier) {
        m_drivetrain = drivetrain;
        m_velocityXSupplier = velocityXSupplier;
        m_velocityYSupplier = velocityYSupplier;
        m_deadbandSupplier = deadbandSupplier;
        m_cageSupplier = cageSupplier;
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
                                            driveFacingAngle.resetProfile();
                                            m_drivetrain.setControl(driveFacingAngle
                                                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                                                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                                                    .withTargetDirection(Aiming.nearestRotation(
                                                            m_drivetrain
                                                                    .getState()
                                                                    .Pose
                                                                    .getRotation(),
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
                                    driveToPosition.resetProfile();
                                    Pose2d currentPose = m_drivetrain.getState().Pose;
                                    Alliance alliance =
                                            DriverStation.getAlliance().orElseThrow();
                                    Pose2d treePose;
                                    if (alliance == Alliance.Blue) {
                                        treePose = currentPose.nearest(FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS);
                                    } else {
                                        treePose = currentPose.nearest(FieldConstants.RED_REEF_TREE_AIMING_POSITIONS);
                                    }
                                    m_drivetrain.setControl(driveToPosition.withTargetPose(treePose));
                                },
                                () -> m_drivetrain.setControl(driveToPosition)))
                .finallyDo(() -> m_drivetrain.updateVisionTarget(false));
    }

    public Command driveToTreeSimple() {
        return m_drivetrain
                .startRun(
                        () -> {
                            m_drivetrain.updateVisionTarget(true);
                            driveToPosition.resetProfile();
                            Pose2d currentPose = m_drivetrain.getState().Pose;
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
                            Pose2d treePose;
                            if (alliance == Alliance.Blue) {
                                treePose = currentPose.nearest(FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS);
                            } else {
                                treePose = currentPose.nearest(FieldConstants.RED_REEF_TREE_AIMING_POSITIONS);
                            }
                            m_drivetrain.setControl(driveToPosition.withTargetPose(treePose));
                        },
                        () -> m_drivetrain.setControl(driveToPosition))
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
                            driveToPosition.resetProfile();
                            Pose2d currentPose = m_drivetrain.getState().Pose;
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
                            Pose2d coralStationPose;
                            if (alliance == Alliance.Blue) {
                                coralStationPose = currentPose.nearest(FieldConstants.BLUE_CORAL_STATION_POSES);
                            } else {
                                coralStationPose = currentPose.nearest(FieldConstants.RED_CORAL_STATION_POSES);
                            }
                            m_drivetrain.setControl(driveToPosition.withTargetPose(coralStationPose));
                        },
                        () -> m_drivetrain.setControl(driveToPosition)));
    }

    /**
     * If the robot is on it's alliance's side, it will drive in front of the cage, then forwards/behind the cage, then to the cage's center.
     * If the robot is on the opposing alliance's side, it will drive behind the cage, then forwards/in front of the cage, then to the cage's center.
     */
    public Command driveIntoCage() {
        return Commands.either(
                Commands.sequence(
                        driveNearCageCommand(true, true, false),
                        driveNearCageCommand(false, false, false),
                        driveIntoCageCommand(false)),
                Commands.sequence(
                        driveNearCageCommand(true, false, true),
                        driveNearCageCommand(false, true, true),
                        driveIntoCageCommand(true)),
                () -> {
                    Translation2d robotPosition = m_drivetrain.getState().Pose.getTranslation();
                    Alliance alliance = DriverStation.getAlliance().orElseThrow();
                    return (alliance == Alliance.Blue
                                    && robotPosition.getX() < FieldConstants.BARGE_CENTER_X_DISTANCE.in(Meters))
                            || (alliance == Alliance.Red
                                    && robotPosition.getX() > FieldConstants.BARGE_CENTER_X_DISTANCE.in(Meters));
                });
    }

    /**
     * Creates a command that drives up to the cage with PathPlanner.
     * @param driveFast True if you want to drive at full speed, false if you want to be slow (like when catching the cage)
     * @param inFront True if you want to drive before the cage, false if you want to drive past it (according to driver station perspective).
     * @param facingLeft True if you want the robot to face left, false if you want it to face right (according to driver station perspective).
     * @return A deferred command that is not created until it is executed.
     */
    private Command driveNearCageCommand(boolean driveFast, boolean inFront, boolean facingLeft) {
        return m_drivetrain.defer(() -> {
            CageLocation cage = m_cageSupplier.get();

            Translation2d cagePosition;
            Translation2d targetPosition;
            Rotation2d targetRotation;

            if (cage == CageLocation.LEFT) {
                cagePosition = FieldConstants.BLUE_CAGE_POSITIONS.get(2);
            } else if (cage == CageLocation.MIDDLE) {
                cagePosition = FieldConstants.BLUE_CAGE_POSITIONS.get(1);
            } else {
                cagePosition = FieldConstants.BLUE_CAGE_POSITIONS.get(0);
            }
            if (inFront) {
                targetPosition = cagePosition.minus(FieldConstants.BARGE_TAPE_DISTANCE);
            } else {
                targetPosition = cagePosition.plus(FieldConstants.BARGE_TAPE_DISTANCE);
            }
            if (facingLeft) {
                targetRotation = Rotation2d.kCCW_90deg;
            } else {
                targetRotation = Rotation2d.kCW_90deg;
            }

            PathConstraints pathConstraints;
            if (driveFast == true) {
                pathConstraints = ON_THE_FLY_CONSTRAINTS;
            } else {
                pathConstraints = CAGE_CONSTRAINTS;
            }

            return generatePath(
                    m_drivetrain.getState(), new Pose2d(targetPosition, targetRotation), pathConstraints, true);
        });
    }

    /**
     * Creates a command that drives into the cage with PathPlanner.
     * @param facingLeft True if you want the robot to face left, false if you want it to face right (according to driver station perspective).
     * @return A deferred command that is not created until it is executed.
     */
    private Command driveIntoCageCommand(boolean facingLeft) {
        return m_drivetrain.defer(() -> {
            CageLocation cage = m_cageSupplier.get();

            Translation2d cagePosition;
            Rotation2d targetRotation;

            if (cage == CageLocation.LEFT) {
                cagePosition = FieldConstants.BLUE_CAGE_POSITIONS.get(2);
            } else if (cage == CageLocation.MIDDLE) {
                cagePosition = FieldConstants.BLUE_CAGE_POSITIONS.get(1);
            } else {
                cagePosition = FieldConstants.BLUE_CAGE_POSITIONS.get(0);
            }
            if (facingLeft) {
                targetRotation = Rotation2d.kCCW_90deg;
            } else {
                targetRotation = Rotation2d.kCW_90deg;
            }

            return generatePath(
                    m_drivetrain.getState(), new Pose2d(cagePosition, targetRotation), CAGE_CONSTRAINTS, true);
        });
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
        Rotation2d currentDirectionOfTravel =
                new Rotation2d(currentState.Speeds.vxMetersPerSecond, currentState.Speeds.vyMetersPerSecond);

        Translation2d targetPosition = targetPose.getTranslation();
        Rotation2d targetRotation = targetPose.getRotation();

        if (allowTargetFlipping && DriverStation.getAlliance().orElseThrow().equals(Alliance.Red)) {
            targetPosition = FlippingUtil.flipFieldPosition(targetPosition);
            targetRotation = FlippingUtil.flipFieldRotation(targetRotation);
        }

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentState.Pose.getX(), currentState.Pose.getY(), currentDirectionOfTravel),
                new Pose2d(targetPosition.getX(), targetPosition.getY(), targetRotation));

        IdealStartingState startingState = new IdealStartingState(
                Math.hypot(currentState.Speeds.vxMetersPerSecond, currentState.Speeds.vyMetersPerSecond),
                currentState.Pose.getRotation());

        PathPlannerPath path =
                new PathPlannerPath(waypoints, pathConstraints, startingState, new GoalEndState(0, targetRotation));
        path.preventFlipping = true;

        // System.out.println("Target x: " + Units.metersToInches(treePose.getX()) + "\nTarget y: "
        //         + Units.metersToInches(treePose.getY()) + "\nTarget direction: "
        //         + treePose.getRotation().getDegrees());

        return AutoBuilder.followPath(path);
    }
}
