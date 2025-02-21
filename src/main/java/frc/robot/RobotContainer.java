// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.K_ANGULAR_D;
import static frc.robot.subsystems.drive.DriveConstants.K_ANGULAR_P;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.catcher.Catcher;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeArmPositions;

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
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withHeadingPID(K_ANGULAR_P, 0, K_ANGULAR_D);

    private final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.SysIdSwerveRotation angularConstraintsCharacterizer =
            new SwerveRequest.SysIdSwerveRotation().withRotationalRate(DriveConstants.MAX_ANGULAR_RATE);

    /* Controllers */
    private final CommandPS4Controller driverController =
            new CommandPS4Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final AimingRoutines aimingRoutines =
            new AimingRoutines(m_drivetrain, this::getVelocityX, this::getVelocityY, this::getDeadband);

    /* Robot States */
    private boolean m_slowMode = false;

    private final DriveTelemetry driveTelemetry = new DriveTelemetry();

    /* NetworkTables Logging */
    // private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // private final NetworkTable driverInfoTable = inst.getTable("Driver Info");

    // private final StringPublisher selectedDirectionIndicator =
    //         driverInfoTable.getStringTopic("Selected Tree Direction").publish();

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
        // selectLeftTree();
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
     * please use <a href="
     * https://www.padcrafter.com/?templates=Driver+Controller&plat=1&leftStick=Drive&aButton=Lock+on+to+reef&xButton=&yButton=&leftBumper=Face+Left+Coral+Station&backButton=Reset+robot+orientation&rightBumper=Face+Right+Coral+Station&bButton=&leftTrigger=Slow+Mode&rightTrigger=Fast+Mode&dpadLeft=Point+wheels+with+right+joystick&rightStick=
     * ">this controller map</a>
     * to update and view the current controls.
     */
    private void configureDriverControls() {
        driverController.square().onTrue(aimingRoutines.setTargetPoseToCurrentPose());
        driverController.triangle().whileTrue(aimingRoutines.driveToPositionTest());

        // driverController.povUp().whileTrue(m_drivetrain.applyRequest(() -> drive.withVelocityX(
        //                 DriveConstants.MAX_LINEAR_SPEED)
        //         .withVelocityY(0)
        //         .withRotationalRate(0)
        //         .withDeadband(getDeadband())
        //         .withRotationalDeadband(getRotationalDeadband())));

        // driverController.povRight().whileTrue(m_drivetrain.applyRequest(() -> angularConstraintsCharacterizer));

        // Slow Mode
        driverController.L2().onTrue(Commands.runOnce(() -> this.m_slowMode = true));
        // Fast Mode
        driverController.R2().onTrue(Commands.runOnce(() -> this.m_slowMode = false));
        // Select left tree
        driverController.L2().onTrue(aimingRoutines.alignWithStation(true));
        // Select right tree
        driverController.R1().onTrue(aimingRoutines.alignWithStation(false));

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

        driverController.circle().toggleOnTrue(aimingRoutines.orientToFaceReefWall());
        driverController.povRight().toggleOnTrue(m_drivetrain.applyRequest(() -> driveFacingAngle
                .withVelocityX(getVelocityX())
                .withVelocityY(getVelocityY())
                .withTargetDirection(Rotation2d.kCCW_90deg)
                .withDeadband(getDeadband())));

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
     * please use <a href="https://www.padcrafter.com/index.php?templates=Operator%20Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&rightTrigger=Control%20Climb%20with%20joysticks&leftTrigger=Control%20both%20Climb%20with%20left%20joystick&leftBumper=Control%20catcher%20arm%20with%20left%20joystick&rightBumper=Control%20intake%20arm%20with%20left%20joystick&aButton=Intake%20coral&bButton=Spit%20out%20coral&xButton=Intake%20algae&yButton=Spit%20out%20algae">this controller map</a>
     * to update and view the current controls.
     */
    private void configureOperatorControls() {
        operatorController
                .rightTrigger()
                .whileTrue(m_climb.motorTestCommand(
                        () -> -operatorController.getLeftY(), () -> -operatorController.getRightY()));
        operatorController
                .leftTrigger()
                .whileTrue(m_climb.motorTestCommand(
                        () -> -operatorController.getLeftY(), () -> -operatorController.getLeftY()));
        operatorController.leftBumper().whileTrue(m_catcher.armTestCommand(() -> -operatorController.getLeftY()));
        operatorController.rightBumper().whileTrue(m_intake.armTestCommand(() -> -operatorController.getLeftY()));
        operatorController.a().whileTrue(m_catcher.wheelTestCommand());
        operatorController.b().whileTrue(m_catcher.wheelBackwardsTestCommand());
        // operatorController.x().whileTrue(m_catcher.wheelTestSlowCommand());
        operatorController.x().whileTrue(m_intake.runWheelsCommand());
        operatorController.y().whileTrue(m_intake.wheelsBackwardsTestCommand());

        operatorController.povDown().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.GROUND_PICKUP));
        operatorController.povUp().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.PROCESSOR));
        operatorController.povLeft().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.STOWED));
        // operatorController.povUp().toggleOnTrue(m_catcher.setArmPositionCommand(CatcherArmPositions.CORAL_STATION));
        // operatorController.povDown().toggleOnTrue(m_catcher.setArmPositionCommand(CatcherArmPositions.L_1));
        // operatorController.povRight().toggleOnTrue(m_catcher.setArmPositionCommand(CatcherArmPositions.TROUGH));
    }

    /**
     * Registers the <a href="https://pathplanner.dev/pplib-named-commands.html">Named Commands</a> used in PathPlanner.
     */
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

    /*private void selectLeftTree() {
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
    }*/

    public void resetFieldPosition(Pose2d position) {
        m_drivetrain.resetPose(position);
    }
}
