// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingNearestPosition;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingPosition;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricVisualServoing;
import frc.robot.util.Aiming;
import frc.robot.util.LimelightHelpers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform.
     * Hint: Deadbands are a percentage of the joystick input.
     * 0.1 means you don't want to move until the joystick is pushed at least 10% in any direction (to prevent drift)
     */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

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
                    DriveConstants.ANGULAR_MOTION_CONSTRAINTS)
            .withPIDGains(DriveConstants.K_P, 0, DriveConstants.K_D)
            .withTolerance(DriveConstants.GOAL_TOLERANCE);

    private final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt();

    /* Controllers */
    private final CommandPS4Controller driverController =
            new CommandPS4Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    /* Robot States */
    private volatile boolean slowMode = false;

    /* NetworkTables Logging */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final DriveTelemetry driveTelemetry = new DriveTelemetry();
    private final NetworkTable driverInfoTable = inst.getTable("Driver Info");
    /**
     * Gets the ID of the primary in-view apriltag.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">limelight NetworkTables API</a>
     * @see {@link frc.robot.util.LimelightHelpers#getFiducialID(String) LimelightHelpers equivalent}
     */
    private final IntegerSubscriber apriltagID =
            inst.getTable(VisionConstants.LIMELIGHT_NAME).getIntegerTopic("tid").subscribe(0);

    private final StringPublisher selectedDirectionIndicator =
            driverInfoTable.getStringTopic("Selected Tree Direction").publish();

    /* Autonomous Sequence Selector (open up in a dashboard like Elastic) */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        setDefaultCommands();
        configureTriggers();
        configureDriverControls();
        configureOperatorControls();

        drivetrain.registerTelemetry(driveTelemetry::telemeterize);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Select your auto:", autoChooser);

        // Select left tree on startup
        selectLeftTree();
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(getVelocityX())
                .withVelocityY(getVelocityY())
                .withRotationalRate(getRotationalRate())
                .withDeadband(getDeadband())
                .withRotationalDeadband(getRotationalDeadband())));
    }

    /**
     * Use this method to define trigger->command mappings that don't involve controller inputs.
     */
    private void configureTriggers() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(exampleSubsystem));
    }

    /**
     * Use this method to define controller input->command mappings.
     * please use <a href="https://www.padcrafter.com/?templates=Driver%20Controller&plat=1&leftStick=Drive&aButton=Lock%20on%20to%20tree&xButton&yButton&leftBumper=Select%20Left%20Tree&backButton=Reset%20robot%20orientation&rightBumper=Select%20Right%20Tree&bButton=Lock%20on%20to%20reef&leftTrigger=Slow%20Mode&rightTrigger=Fast%20Mode&dpadLeft=Point%20wheels%20with%20right%20joystick&rightStick">this controller map</a>
     * to update and view the current controls.
     */
    private void configureDriverControls() {
        driverController.povUp().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(
                        DriveConstants.MAX_LINEAR_SPEED * 0.2)
                .withVelocityY(0)
                .withRotationalRate(0)
                .withDeadband(getDeadband())
                .withRotationalDeadband(getRotationalDeadband())));

        // Slow Mode
        driverController.L2().onTrue(Commands.runOnce(() -> this.slowMode = true));
        // Fast Mode
        driverController.R2().onTrue(Commands.runOnce(() -> this.slowMode = false));
        // Select left tree
        driverController.L1().onTrue(Commands.runOnce(this::selectLeftTree));
        // Select right tree
        driverController.R1().onTrue(Commands.runOnce(this::selectRightTree));

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
        driverController
                .circle()
                .toggleOnTrue(drivetrain
                        .runOnce(driveFacingPosition::resetProfile)
                        .andThen(
                                // Face either the blue or red reef
                                drivetrain
                                        .applyRequest(() -> {
                                            Alliance alliance =
                                                    DriverStation.getAlliance().orElse(null);
                                            if (alliance == Alliance.Blue) {
                                                driveFacingPosition.withTargetPosition(FieldConstants.BLUE_REEF_CENTER);
                                            } else if (alliance == Alliance.Red) {
                                                driveFacingPosition.withTargetPosition(FieldConstants.RED_REEF_CENTER);
                                            } else {
                                                DriverStation.reportWarning(
                                                        "Driver Station not connected, robot will drive normally!",
                                                        false);
                                                return drive.withVelocityX(getVelocityX())
                                                        .withVelocityY(getVelocityY())
                                                        .withRotationalRate(getRotationalRate())
                                                        .withDeadband(getDeadband())
                                                        .withRotationalDeadband(getRotationalDeadband());
                                            }
                                            return driveFacingPosition
                                                    .withVelocityX(getVelocityX())
                                                    .withVelocityY(getVelocityY())
                                                    .withDeadband(getDeadband());
                                        })
                                        .until(driveFacingPosition::motionIsFinished))
                        // THIS COMMAND DOES NOT DRIVE. It just updates the target direction variable.
                        .andThen(drivetrain.runOnce(() -> driveFacingAngle.withTargetDirection(Aiming.nearestRotation(
                                drivetrain.getState().Pose.getRotation(), FieldConstants.REEF_WALL_ROTATIONS))))
                        // Drive at a fixed rotation
                        .andThen(drivetrain
                                .applyRequest(() -> driveFacingAngle
                                        .withVelocityX(getVelocityX())
                                        .withVelocityY(getVelocityY())
                                        .withDeadband(getDeadband()))
                                .until(() -> Aiming.isReefTag((int) apriltagID.get())))
                        // Drive facing perpendicular to the apriltag
                        .andThen(drivetrain.applyRequest(() -> {
                            int id = (int) apriltagID.get();
                            if (Aiming.isReefTag(id)) {
                                driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[id - 1]);
                            }
                            return driveFacingAngle
                                    .withVelocityX(getVelocityX())
                                    .withVelocityY(getVelocityY())
                                    .withDeadband(getDeadband());
                        })));

        /*
         * This lengthy sequence is for locking on to a tree. Here is the explanation:
         * Once the driver presses this button, the robot will rotate to face the center of the reef.
         * The point of this is to give the camera a chance to see the correct tag without requiring the driver to rotate manually.
         * Once this movement is finished, it will attempt to lock onto the nearest tree using its position.
         * If a reef tag is in view or enters view, the robot will instead start aiming at an offset from the tag using tx.
         * The offset is to the left if the operator has selected the left tree,
         * or to the right if the operator has selected the right tree.
         * (They can change their selection any time.)
         * If the robot loses sight of all reef tags, it will finish rotating based on the last known tx value,
         * and then stop rotating unless a tag returns in view.
         * This is to provide the driver a chance to drive forwards/backwards/left/right to align the robot to a branch themself.
         */
        driverController
                .cross()
                .toggleOnTrue(drivetrain
                        .runOnce(driveFacingPosition::resetProfile)
                        .andThen(drivetrain
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
                                        return drive.withVelocityX(getVelocityX())
                                                .withVelocityY(getVelocityY())
                                                .withRotationalRate(getRotationalRate())
                                                .withDeadband(getDeadband())
                                                .withRotationalDeadband(getRotationalDeadband());
                                    }
                                    return driveFacingPosition
                                            .withVelocityX(getVelocityX())
                                            .withVelocityY(getVelocityY())
                                            .withDeadband(getDeadband());
                                })
                                .until(driveFacingPosition::motionIsFinished))
                        // Drive facing either the nearest blue tree or nearest red tree
                        .andThen(drivetrain
                                .applyRequest(() -> {
                                    Alliance alliance =
                                            DriverStation.getAlliance().orElse(null);
                                    if (alliance == Alliance.Blue) {
                                        driveFacingNearestPosition.withTargetPositions(
                                                FieldConstants.BLUE_REEF_TREE_POSITIONS);
                                    } else if (alliance == Alliance.Red) {
                                        driveFacingNearestPosition.withTargetPositions(
                                                FieldConstants.RED_REEF_TREE_POSITIONS);
                                    } else {
                                        DriverStation.reportWarning(
                                                "Driver Station not connected, robot will drive normally!", false);
                                        return drive.withVelocityX(getVelocityX())
                                                .withVelocityY(getVelocityY())
                                                .withRotationalRate(getRotationalRate())
                                                .withDeadband(getDeadband())
                                                .withRotationalDeadband(getRotationalDeadband());
                                    }
                                    return driveFacingNearestPosition
                                            .withVelocityX(getVelocityX())
                                            .withVelocityY(getVelocityY())
                                            .withDeadband(getDeadband());
                                })
                                .until(() -> Aiming.isReefTag((int) apriltagID.get())))
                        // Face the middle of the tag
                        .andThen(drivetrain.applyRequest(() -> driveFacingVisionTarget
                                .withVelocityX(getVelocityX())
                                .withVelocityY(getVelocityY())
                                .withDeadband(getDeadband()))));

        // Point wheels with right joystick
        driverController
                .povLeft()
                .whileTrue(drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(
                        new Rotation2d(-driverController.getRightY(), -driverController.getRightX()))));

        // Reset robot orientation
        driverController.touchpad().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        /*
         * Run SysId routines when holding back/start and X/Y.
         * Note that each routine should be run exactly once in a single log.
         * Comment out when finished.
         */
        driverController
                .share()
                .and(driverController.triangle())
                .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverController
                .share()
                .and(driverController.square())
                .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        driverController
                .options()
                .and(driverController.triangle())
                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverController
                .options()
                .and(driverController.square())
                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        operatorController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
    }

    /**
     * Use this method to define controller input->command mappings.
     * please use <a href="https://www.padcrafter.com/index.php?templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF">this controller map</a>
     * to update and view the current controls.
     */
    private void configureOperatorControls() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * X is defined as forward.
     */
    public double getVelocityX() {
        double velocityX = DriveConstants.MAX_LINEAR_SPEED * -driverController.getLeftY();
        if (slowMode) {
            velocityX *= 0.5;
        }
        return velocityX;
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * Y is defined as left.
     */
    public double getVelocityY() {
        double velocityY = DriveConstants.MAX_LINEAR_SPEED * -driverController.getLeftX();
        if (slowMode) {
            velocityY *= 0.5;
        }
        return velocityY;
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * rotation is defined as counterclockwise.
     */
    public double getRotationalRate() {
        double rotationalRate = DriveConstants.MAX_ANGULAR_RATE * -driverController.getRightX();
        if (slowMode) {
            rotationalRate *= 0.5;
        }
        return rotationalRate;
    }

    /**
     * @return The linear deadband in meters per second.
     */
    public double getDeadband() {
        double deadband = DriveConstants.MAX_LINEAR_SPEED * 0.1;
        if (slowMode) {
            deadband *= 0.5;
        }
        return deadband;
    }

    /**
     * @return The rotational deadband in radians per second.
     */
    public double getRotationalDeadband() {
        double rotationalDeadband = DriveConstants.MAX_ANGULAR_RATE * 0.1;
        if (slowMode) {
            rotationalDeadband *= 0.5;
        }
        return rotationalDeadband;
    }

    private void selectLeftTree() {
        // Update the target for tx values
        LimelightHelpers.SetFidcuial3DOffset(
                VisionConstants.LIMELIGHT_NAME,
                VisionConstants.TAG_TO_LEFT_TREE_FORWARD_OFFSET,
                VisionConstants.TAG_TO_LEFT_TREE_RIGHT_OFFSET,
                0);
        // Log the direction
        selectedDirectionIndicator.set("Left");
    }

    private void selectRightTree() {
        // Update the target for tx values
        LimelightHelpers.SetFidcuial3DOffset(
                VisionConstants.LIMELIGHT_NAME,
                VisionConstants.TAG_TO_RIGHT_TREE_FORWARD_OFFSET,
                VisionConstants.TAG_TO_RIGHT_TREE_RIGHT_OFFSET,
                0);
        // Log the direction
        selectedDirectionIndicator.set("Right");
    }

    public void resetFieldPosition(Pose2d position) {
        drivetrain.resetPose(position);
    }
}
