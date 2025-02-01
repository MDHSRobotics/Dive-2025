// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.commands.AimingRoutines;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.Catcher;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.util.LimelightHelpers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final Climb m_climb = new Climb();
    private final Catcher m_catcher = new Catcher();
    private final Intake m_intake = new Intake();

    /* Setting up bindings for necessary control of the swerve drive platform.
     */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.SysIdSwerveRotation angularConstraintsCharacterizer =
            new SwerveRequest.SysIdSwerveRotation().withRotationalRate(DriveConstants.MAX_ANGULAR_RATE);

    /* Controllers */
    private final CommandPS4Controller driverController =
            new CommandPS4Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final AimingRoutines aimingRoutines = new AimingRoutines(
            m_drivetrain,
            drive,
            this::getVelocityX,
            this::getVelocityY,
            this::getRotationalRate,
            this::getDeadband,
            this::getRotationalDeadband);

    /* Robot States */
    private boolean m_slowMode = false;

    /* NetworkTables Logging */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final DriveTelemetry driveTelemetry = new DriveTelemetry();
    private final NetworkTable driverInfoTable = inst.getTable("Driver Info");

    private final StringPublisher selectedDirectionIndicator =
            driverInfoTable.getStringTopic("Selected Tree Direction").publish();

    /* Autonomous Sequence Selector (open up in a dashboard like Elastic) */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        registerNamedCommands();
        setDefaultCommands();
        configureTriggers();
        configureDriverControls();
        configureOperatorControls();

        m_drivetrain.registerTelemetry(driveTelemetry::telemeterize);

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                WheelRadiusCharacterization.characterizationCommand(m_drivetrain));
        SmartDashboard.putData("Select your auto:", autoChooser);

        // Select left tree on startup
        selectLeftTree();

        // m_drivetrain.resetRotation(Rotation2d.fromDegrees(54));
    }

    private void setDefaultCommands() {
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(() -> drive.withVelocityX(getVelocityX())
                .withVelocityY(getVelocityY())
                .withRotationalRate(getRotationalRate())
                .withDeadband(getDeadband())
                .withRotationalDeadband(getRotationalDeadband())));
        m_climb.setDefaultCommand(m_climb.disableMotorsCommand());
        m_catcher.setDefaultCommand(m_catcher.disableMotorsCommand());
        m_intake.setDefaultCommand(m_intake.disableMotorsCommand());
    }

    /**
     * Use this method to define trigger->command mappings that don't involve controller inputs.
     */
    private void configureTriggers() {}

    /**
     * Use this method to define controller input->command mappings.
     * please use <a href="https://www.padcrafter.com/?templates=Driver+Controller&plat=1&leftStick=Drive&aButton=Lock+on+to+tree&xButton=Lock+on+to+tree+%28no+limelight%29&yButton=&leftBumper=Select+Left+Tree&backButton=Reset+robot+orientation&rightBumper=Select+Right+Tree&bButton=Lock+on+to+reef&leftTrigger=Slow+Mode&rightTrigger=Fast+Mode&dpadLeft=Point+wheels+with+right+joystick&rightStick=">this controller map</a>
     * to update and view the current controls.
     */
    private void configureDriverControls() {
        /*driverController.povUp().whileTrue(m_drivetrain.applyRequest(() -> drive.withVelocityX(
                DriveConstants.MAX_LINEAR_SPEED)
        .withVelocityY(0)
        .withRotationalRate(0)
        .withDeadband(getDeadband())
        .withRotationalDeadband(getRotationalDeadband())));*/

        // driverController.povRight().whileTrue(m_drivetrain.applyRequest(() -> angularConstraintsCharacterizer));

        // Slow Mode
        driverController.L2().onTrue(Commands.runOnce(() -> this.m_slowMode = true));
        // Fast Mode
        driverController.R2().onTrue(Commands.runOnce(() -> this.m_slowMode = false));
        // Select left tree
        driverController.L1().onTrue(Commands.runOnce(this::selectLeftTree));
        // Select right tree
        driverController.R1().onTrue(Commands.runOnce(this::selectRightTree));

        // Point wheels with right joystick
        driverController
                .povLeft()
                .whileTrue(m_drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(
                        new Rotation2d(-driverController.getRightY(), -driverController.getRightX()))));

        // Reset robot orientation
        driverController
                .touchpad()
                .onTrue(m_drivetrain.runOnce(() -> m_drivetrain.setOperatorPerspectiveForward(
                        m_drivetrain.getState().Pose.getRotation())));

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
        driverController.circle().toggleOnTrue(aimingRoutines.orientToFaceReefWall());

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
        driverController.cross().toggleOnTrue(aimingRoutines.orientToFaceTree());

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
        driverController.square().toggleOnTrue(aimingRoutines.orientToFaceTreeWithoutLimelight());

        /*
         * Run SysId routines when holding back/start and X/Y.
         * Note that each routine should be run exactly once in a single log.
         * Comment out when finished.
         */
        driverController
                .share()
                .and(driverController.povUp())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverController
                .share()
                .and(driverController.povDown())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        driverController
                .options()
                .and(driverController.povUp())
                .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverController
                .options()
                .and(driverController.povDown())
                .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this method to define controller input->command mappings.
     * please use <a href="https://www.padcrafter.com/index.php?templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF">this controller map</a>
     * to update and view the current controls.
     */
    private void configureOperatorControls() {
        /*operatorController
                .rightTrigger()
                .whileTrue(m_climb.motorTestCommand(
                        () -> -operatorController.getLeftY(), () -> -operatorController.getRightY()));
        operatorController
                .leftTrigger()
                .whileTrue(m_climb.motorTestCommand(
                        () -> -operatorController.getLeftY(), () -> -operatorController.getLeftY()));
        operatorController
                .rightBumper()
                .whileTrue(m_climb.motorTestCommand(
                        () -> -operatorController.getLeftY(), () -> -operatorController.getLeftY()));*/
        operatorController.rightTrigger().whileTrue(m_intake.armTestCommand(() -> -operatorController.getLeftY()));
    }

    private void registerNamedCommands() {
        // NamedCommands.registerCommand("Lower Arm", null);
    }

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
        if (m_slowMode) {
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
        if (m_slowMode) {
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
        if (m_slowMode) {
            rotationalRate *= 0.5;
        }
        return rotationalRate;
    }

    /**
     * Deadbands are a percentage of the joystick input.
     * 0.1 means you don't want to move until the joystick is pushed at least 10% in any direction (to prevent drift)
     * @return The linear deadband in meters per second.
     */
    public double getDeadband() {
        double deadband = DriveConstants.MAX_LINEAR_SPEED * 0.1;
        if (m_slowMode) {
            deadband *= 0.5;
        }
        return deadband;
    }

    /**
     * Deadbands are a percentage of the joystick input.
     * 0.1 means you don't want to move until the joystick is pushed at least 10% in any direction (to prevent drift)
     * @return The rotational deadband in radians per second.
     */
    public double getRotationalDeadband() {
        double rotationalDeadband = DriveConstants.MAX_ANGULAR_RATE * 0.1;
        if (m_slowMode) {
            rotationalDeadband *= 0.5;
        }
        return rotationalDeadband;
    }

    private void selectLeftTree() {
        // Update the target for tx values
        LimelightHelpers.SetFidcuial3DOffset(
                VisionConstants.FRONT_LIMELIGHT_NAME,
                VisionConstants.TAG_TO_LEFT_TREE_FORWARD_OFFSET,
                VisionConstants.TAG_TO_LEFT_TREE_RIGHT_OFFSET,
                0);
        // Log the direction
        selectedDirectionIndicator.set("Left");
    }

    private void selectRightTree() {
        // Update the target for tx values
        LimelightHelpers.SetFidcuial3DOffset(
                VisionConstants.FRONT_LIMELIGHT_NAME,
                VisionConstants.TAG_TO_RIGHT_TREE_FORWARD_OFFSET,
                VisionConstants.TAG_TO_RIGHT_TREE_RIGHT_OFFSET,
                0);
        // Log the direction
        selectedDirectionIndicator.set("Right");
    }

    public void resetFieldPosition(Pose2d position) {
        m_drivetrain.resetPose(position);
    }
}
