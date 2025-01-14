// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private boolean slowMode = false;

    /* NetworkTables Logging */
    private final DriveTelemetry driveTelemetry = new DriveTelemetry();
    private final NetworkTable driverInfoTable =
            NetworkTableInstance.getDefault().getTable("Driver Info");

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
     * please use <a href="https://www.padcrafter.com/?templates=Driver+Controller&plat=1&leftStick=Drive&aButton=Lock+on+to+tree&xButton=&yButton=&leftBumper=Select+Left+Tree&backButton=Reset+robot+orientation&rightBumper=Select+Right+Tree&bButton=Point+wheels+with+left+joystick&leftTrigger=Slow+Mode&rightTrigger=Fast+Mode">this controller map</a>
     * to update and view the current controls.
     */
    private void configureDriverControls() {
        // Slow Mode
        driverController.L2().onTrue(Commands.runOnce(() -> this.slowMode = true));
        // Fast Mode
        driverController.R2().onTrue(Commands.runOnce(() -> this.slowMode = false));
        // Select left tree
        driverController.L1().onTrue(Commands.runOnce(this::selectLeftTree));
        // Select right tree
        driverController.R1().onTrue(Commands.runOnce(this::selectRightTree));

        /*
         * This very lengthy sequence is for locking on to a tree. Here is the explanation:
         * Once the driver presses this button, the robot will rotate to face the center of the reef.
         * The point of this is to give the camera a chance to see the correct tag without requiring the driver to rotate manually.
         * Once this movement is finished, it will attempt to lock onto the nearest tree.
         * If a tag is in view or enters view, the robot will instead start aiming at an offset from the tag using tx.
         * The offset is to the left if the operator has selected the left tree,
         * or to the right if the operator has selected the right tree.
         * (They can change their selection any time.)
         * If a tag is no longer in view, the robot will finish rotating based on the last known tx value,
         * and then stop rotating unless a tag returns in view.
         * This is to provide the driver a chance to drive forwards/backwards/left/right to align the robot to a branch themself.
         */
        driverController
                .cross()
                .toggleOnTrue(drivetrain
                        .runOnce(driveFacingPosition::resetProfile)
                        .andThen(drivetrain.applyRequest(() -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(null);
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
                                    .withDeadband(getDeadband())
                                    .withRotationalDeadband(getRotationalDeadband());
                        }))
                        .until(driveFacingPosition::motionIsFinished)
                        .andThen(drivetrain.runOnce(driveFacingNearestPosition::resetProfile))
                        .andThen(drivetrain.applyRequest(() -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(null);
                            if (alliance == Alliance.Blue) {
                                driveFacingNearestPosition.withTargetPositions(FieldConstants.BLUE_REEF_TREE_POSITIONS);
                            } else if (alliance == Alliance.Red) {
                                driveFacingNearestPosition.withTargetPositions(FieldConstants.RED_REEF_TREE_POSITIONS);
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
                                    .withDeadband(getDeadband())
                                    .withRotationalDeadband(getRotationalDeadband());
                        }))
                        .until(() -> LimelightHelpers.getFiducialID(VisionConstants.LIMELIGHT_NAME) != 0)
                        .andThen(drivetrain.runOnce(driveFacingVisionTarget::resetProfile))
                        .andThen(drivetrain.applyRequest(() -> driveFacingVisionTarget
                                .withVelocityX(getVelocityX())
                                .withVelocityY(getVelocityY())
                                .withDeadband(getDeadband())
                                .withRotationalDeadband(getRotationalDeadband()))));

        // Point wheels with left joystick
        driverController
                .circle()
                .whileTrue(drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

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
}
