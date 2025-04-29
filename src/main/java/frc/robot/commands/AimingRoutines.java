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
import edu.wpi.first.math.MathUtil;
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

    // Joystick suppliers
    private final DoubleSupplier m_leftYSupplier;
    private final DoubleSupplier m_leftXSupplier;
    private final DoubleSupplier m_rightYSupplier;
    private final DoubleSupplier m_rightXSupplier;

    // NetworkTables
    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_cameraTable = m_inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME);

    // Swerve requests
    private final DriveFacingAngle m_driveFacingAngle = new DriveFacingAngle(ROTATION_PID.kP, MAX_ANGULAR_RATE)
            .withTolerance(HEADING_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final DriveFacingPosition m_driveFacingPosition = new DriveFacingPosition(ROTATION_PID.kP, MAX_ANGULAR_RATE)
            .withTolerance(HEADING_TOLERANCE)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final DriveToPose m_driveToPose = new DriveToPose(
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
     * The current target by whichever command is running.
     * You don't need to worry about multiple commands accessing this because only one command can run at a time.
     */
    private Pose2d m_currentTargetPose;

    /**
     * Gets the ID of the primary in-view apriltag.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">limelight NetworkTables API</a>
     * @see {@link frc.robot.util.LimelightHelpers#getFiducialID(String) LimelightHelpers equivalent}
     */
    private final IntegerSubscriber m_apriltagIDSub =
            m_cameraTable.getIntegerTopic("tid").subscribe(0);

    /**
     * Constructs an object that provides <a href="https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories">instanced command factories</a> for swerve drive aiming.
     * @param drivetrain The drivetrain to drive and aim with.
     * @param velocityXSupplier A method reference or lambda that returns X velocity.
     * @param velocityYSupplierA method reference or lambda that returns Y velocity.
     * @param deadbandSupplier A method reference or lambda that returns deadband.
     * @param leftYSupplier A method reference or lambda that returns a left joystick's Y value.
     * @param leftXSupplier A method reference or lambda that returns a left joystick's X value.
     * @param rightYSupplier A method reference or lambda that returns a right joystick's Y value.
     * @param rightXSupplier A method reference or lambda that returns a right joystick's X value.
     */
    public AimingRoutines(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            DoubleSupplier deadbandSupplier,
            DoubleSupplier leftYSupplier,
            DoubleSupplier leftXSupplier,
            DoubleSupplier rightYSupplier,
            DoubleSupplier rightXSupplier) {
        m_drivetrain = drivetrain;
        m_velocityXSupplier = velocityXSupplier;
        m_velocityYSupplier = velocityYSupplier;
        m_deadbandSupplier = deadbandSupplier;
        m_leftYSupplier = leftYSupplier;
        m_leftXSupplier = leftXSupplier;
        m_rightYSupplier = rightYSupplier;
        m_rightXSupplier = rightXSupplier;
    }

    public Command alignWithCoralStation(boolean leftStation) {
        return m_drivetrain.applyResettableRequest(() -> {
            Alliance alliance = DriverStation.getAlliance().orElseThrow();
            if (leftStation) {
                if (alliance == Alliance.Blue) {
                    m_driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[13]);
                } else if (alliance == Alliance.Red) {
                    m_driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[1]);
                }
            } else {
                if (alliance == Alliance.Blue) {
                    m_driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[12]);
                } else if (alliance == Alliance.Red) {
                    m_driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[2]);
                }
            }

            return m_driveFacingAngle
                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                    .withDeadband(m_deadbandSupplier.getAsDouble());
        });
    }

    public Command alignWithProcessor() {
        return m_drivetrain.applyResettableRequest(() -> {
            Alliance alliance = DriverStation.getAlliance().orElseThrow();
            if (alliance == Alliance.Blue) {
                m_driveFacingAngle.withTargetDirection(Rotation2d.kCCW_90deg);
            } else {
                m_driveFacingAngle.withTargetDirection(Rotation2d.kCW_90deg);
            }

            return m_driveFacingAngle
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
                                        m_driveFacingPosition.withTargetPosition(FieldConstants.BLUE_REEF_CENTER);
                                    } else if (alliance == Alliance.Red) {
                                        m_driveFacingPosition.withTargetPosition(FieldConstants.RED_REEF_CENTER);
                                    }
                                    return m_driveFacingPosition
                                            .withVelocityX(m_velocityXSupplier.getAsDouble())
                                            .withVelocityY(m_velocityYSupplier.getAsDouble())
                                            .withDeadband(m_deadbandSupplier.getAsDouble());
                                })
                                .until(m_driveFacingPosition::motionIsFinished),
                        m_drivetrain
                                .startRun(
                                        () -> {
                                            m_driveFacingAngle.resetRequest();
                                            m_driveFacingAngle.withTargetDirection(Aiming.nearestRotation(
                                                    m_drivetrain.getState().Pose.getRotation(),
                                                    FieldConstants.REEF_WALL_ROTATIONS));
                                        },
                                        () -> m_drivetrain.setControl(m_driveFacingAngle
                                                .withVelocityX(m_velocityXSupplier.getAsDouble())
                                                .withVelocityY(m_velocityYSupplier.getAsDouble())
                                                .withDeadband(m_deadbandSupplier.getAsDouble())))
                                .until(() -> Aiming.isReefTag((int) m_apriltagIDSub.get())),
                        m_drivetrain.applyResettableRequest(() -> {
                            int id = (int) m_apriltagIDSub.get();
                            if (Aiming.isReefTag(id)) {
                                m_driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[id]);
                            }
                            return m_driveFacingAngle
                                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                                    .withDeadband(m_deadbandSupplier.getAsDouble());
                        }))
                .finallyDo(() -> m_drivetrain.updateVisionTarget(false));
    }

    /**
     * This command will generate a path to the tree selected by the joysticks and follow it.
     * Once it finishes following the path, it will attempt to correct any inaccuracies in its position until interrupted.
     * See "diagrams/Tree_Selection.jpeg" and "diagrams/Useful_angles_in_radians.png" in the project files for more info.
     */
    public Command driveToTree() {
        return Commands.sequence(
                        m_drivetrain.defer(() -> {
                            m_drivetrain.updateVisionTarget(true);
                            // Get the left joystick angle in the range of 0 to 2*PI
                            final double leftStickAngleRadians = MathUtil.inputModulus(
                                    Math.atan2(-m_leftYSupplier.getAsDouble(), m_leftXSupplier.getAsDouble()),
                                    0.0,
                                    2.0 * Math.PI);

                            // Get the right joystick angle in the range of 0 to 2*PI
                            final double rightStickAngleRadians = MathUtil.inputModulus(
                                    Math.atan2(-m_rightYSupplier.getAsDouble(), m_rightXSupplier.getAsDouble()),
                                    0.0,
                                    2.0 * Math.PI);

                            // Whether or not the joystick chose the left tree
                            final boolean leftTreeSelected = (rightStickAngleRadians >= Math.PI / 2.0)
                                    && (rightStickAngleRadians < 3.0 * Math.PI / 2.0);

                            if (leftStickAngleRadians >= Math.PI / 3.0
                                    && leftStickAngleRadians < 2.0 * Math.PI / 3.0) { // Bottom reef side
                                if (leftTreeSelected) {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(0);
                                } else {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(1);
                                }
                            } else if (leftStickAngleRadians >= 2.0 * Math.PI / 3.0
                                    && leftStickAngleRadians < Math.PI) { // Bottom right reef side
                                if (leftTreeSelected) {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(2);
                                } else {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(3);
                                }
                            } else if (leftStickAngleRadians >= Math.PI
                                    && leftStickAngleRadians < 4.0 * Math.PI / 3.0) { // Top right reef side
                                if (leftTreeSelected) {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(4);
                                } else {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(5);
                                }
                            } else if (leftStickAngleRadians >= 4.0 * Math.PI / 3.0
                                    && leftStickAngleRadians < 5.0 * Math.PI / 3.0) { // Top reef side
                                if (leftTreeSelected) {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(6);
                                } else {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(7);
                                }
                            } else if (leftStickAngleRadians >= 5.0 * Math.PI / 3.0
                                    && leftStickAngleRadians < 2.0 * Math.PI) { // Top left reef side
                                if (leftTreeSelected) {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(8);
                                } else {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(9);
                                }
                            } else { // Bottom left reef side
                                if (leftTreeSelected) {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(10);
                                } else {
                                    m_currentTargetPose = FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS.get(11);
                                }
                            }
                            return generatePath(
                                    m_drivetrain.getState(), m_currentTargetPose, ON_THE_FLY_CONSTRAINTS, true);
                        }),
                        positionCorrectionCommand())
                .finallyDo(() -> m_drivetrain.updateVisionTarget(false));
    }

    /**
     * This command will generate a path to the nearest tree and follow it.
     * Once it finishes following the path, it will attempt to correct any inaccuracies in its position until interrupted.
     */
    public Command driveToNearestTree() {
        return Commands.sequence(
                        m_drivetrain.defer(() -> {
                            m_drivetrain.updateVisionTarget(true);
                            SwerveDriveState currentState = m_drivetrain.getState();
                            Pose2d currentPose = m_drivetrain.getState().Pose;
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
                            if (alliance == Alliance.Blue) {
                                m_currentTargetPose =
                                        currentPose.nearest(FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS);
                            } else {
                                m_currentTargetPose =
                                        currentPose.nearest(FieldConstants.RED_REEF_TREE_AIMING_POSITIONS);
                            }
                            return generatePath(currentState, m_currentTargetPose, ON_THE_FLY_CONSTRAINTS, false);
                        }),
                        positionCorrectionCommand())
                .finallyDo(() -> m_drivetrain.updateVisionTarget(false));
    }

    /**
     * An alternative to {@link frc.robot.commands.AimingRoutines#driveToNearestTree() driveToNearestTree()}
     * that is not as smooth and may hit the reef wall, but does not have any startup lag.
     */
    public Command driveToNearestTreeSimple() {
        return m_drivetrain
                .startRun(
                        () -> {
                            m_drivetrain.updateVisionTarget(true);
                            m_driveToPose.resetRequest();
                            Pose2d currentPose = m_drivetrain.getState().Pose;
                            Alliance alliance = DriverStation.getAlliance().orElseThrow();
                            if (alliance == Alliance.Blue) {
                                m_currentTargetPose =
                                        currentPose.nearest(FieldConstants.BLUE_REEF_TREE_AIMING_POSITIONS);
                            } else {
                                m_currentTargetPose =
                                        currentPose.nearest(FieldConstants.RED_REEF_TREE_AIMING_POSITIONS);
                            }
                            m_driveToPose.withTargetPose(m_currentTargetPose);
                        },
                        () -> m_drivetrain.setControl(m_driveToPose))
                .finallyDo(() -> m_drivetrain.updateVisionTarget(false));
    }

    public Command driveToNearestCoralStation() {
        return Commands.sequence(
                m_drivetrain.defer(() -> {
                    SwerveDriveState currentState = m_drivetrain.getState();
                    Pose2d currentPose = m_drivetrain.getState().Pose;
                    Alliance alliance = DriverStation.getAlliance().orElseThrow();
                    if (alliance == Alliance.Blue) {
                        m_currentTargetPose = currentPose.nearest(FieldConstants.BLUE_CORAL_STATION_POSES);
                    } else {
                        m_currentTargetPose = currentPose.nearest(FieldConstants.RED_CORAL_STATION_POSES);
                    }
                    return generatePath(currentState, m_currentTargetPose, CORAL_STATION_CONSTRAINTS, false);
                }),
                positionCorrectionCommand());
    }

    /**
     * @return A command that attempts to get closer to the currently set target pose.
     */
    private Command positionCorrectionCommand() {
        return m_drivetrain.startRun(
                () -> {
                    m_driveToPose.resetRequest();
                    m_driveToPose.withTargetPose(m_currentTargetPose);
                },
                () -> m_drivetrain.setControl(m_driveToPose));
    }

    /**
     * Generates a path-following command that drives the robot to a target position and rotation.
     * <p>
     * The driver must make sure to NEVER be driving straight away from the target when this method is called,
     * or else it will generate a path that does not respect the robot's momentum.
     * See "diagrams/Backwards_Curve_With_Initial_Velocity.gif" for a visualization of this situation.
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
