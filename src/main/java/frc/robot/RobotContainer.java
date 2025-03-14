// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.AimingRoutines;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.catcher.Catcher;
import frc.robot.subsystems.catcher.Catcher.CatcherArmPositions;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeArmPositions;
import frc.robot.util.AutoTimer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private enum RobotSpeeds {
        MAX_SPEED,
        HALF_SPEED,
        QUARTER_SPEED
    }

    public enum CageLocation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    // The robot's subsystems and commands are defined here...
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final Climb m_climb = new Climb();
    private final Catcher m_catcher = new Catcher();
    private final Intake m_intake = new Intake();

    private final AutoTimer m_autoTimer = new AutoTimer();

    /* Setting up bindings for necessary control of the swerve drive platform.
     */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    // private final SwerveRequest.SysIdSwerveRotation angularConstraintsCharacterizer =
    //         new SwerveRequest.SysIdSwerveRotation().withRotationalRate(DriveConstants.MAX_ANGULAR_RATE);

    /* Controllers */
    private final CommandPS4Controller driverController =
            new CommandPS4Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    /* Robot States */
    // private final Trigger m_nearCoralStationTrigger = new Trigger(
    // () -> Aiming.isNearCoralStation(m_drivetrain.getState().Pose.getTranslation()));
    private RobotSpeeds m_robotSpeed = RobotSpeeds.MAX_SPEED;

    private final DriveTelemetry driveTelemetry = new DriveTelemetry();

    /* NetworkTables Logging */
    // private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // private final NetworkTable driverInfoTable = inst.getTable("Driver Info");

    // private final StringPublisher selectedDirectionIndicator =
    //         driverInfoTable.getStringTopic("Selected Tree Direction").publish();

    /* Selectors (open up in a dashboard like Elastic) */
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<CageLocation> cageChooser = new SendableChooser<CageLocation>();

    private final AimingRoutines aimingRoutines = new AimingRoutines(
            m_drivetrain, this::getVelocityX, this::getVelocityY, this::getDeadband, cageChooser::getSelected);

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
        autoChooser.addOption("Drive to nearest tree", aimingRoutines.driveToTree());
        autoChooser.addOption("Drive into cage", aimingRoutines.driveIntoCage());
        SmartDashboard.putData("Select your auto:", autoChooser);

        cageChooser.addOption("Left", CageLocation.LEFT);
        cageChooser.addOption("Middle", CageLocation.MIDDLE);
        cageChooser.addOption("Right", CageLocation.RIGHT);
        SmartDashboard.putData("Select your cage:", cageChooser);

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
    private void configureTriggers() {
        // m_nearCoralStationTrigger.onTrue(Commands.runOnce(() -> m_robotSpeed = RobotSpeeds.HALF_SPEED));
        // m_nearCoralStationTrigger.onFalse(Commands.runOnce(() -> m_robotSpeed = RobotSpeeds.MAX_SPEED));
    }

    /**
     * Use this method to define controller input->command mappings.
     * please use <a href="
     * https://www.padcrafter.com/?templates=Driver+Controller&plat=1&leftStick=Drive&aButton=Lock+on+to+reef&xButton=&yButton=Lock+on+to+processor&leftBumper=Face+Left+Coral+Station&backButton=Reset+robot+orientation&rightBumper=Face+Right+Coral+Station&bButton=&leftTrigger=Slow+Mode&rightTrigger=Super+Slow+Mode&dpadLeft=&rightStick=Rotate
     * ">this controller map</a>
     * to update and view the current controls.
     */
    private void configureDriverControls() {
        // driverController.square().onTrue(aimingRoutines.setTargetPoseToCurrentPose());
        // driverController.triangle().whileTrue(aimingRoutines.driveToPositionTest());

        // driverController.povUp().whileTrue(m_drivetrain.applyRequest(() -> drive.withVelocityX(
        //                 DriveConstants.MAX_LINEAR_SPEED)
        //         .withVelocityY(0)
        //         .withRotationalRate(0)
        //         .withDeadband(getDeadband())
        //         .withRotationalDeadband(getRotationalDeadband())));

        // driverController.povUp().whileTrue(m_drivetrain.applyRequest(() -> angularConstraintsCharacterizer));

        // Half Speed
        driverController.L2().onTrue(Commands.runOnce(() -> m_robotSpeed = RobotSpeeds.HALF_SPEED));
        driverController.L2().onFalse(Commands.runOnce(() -> m_robotSpeed = RobotSpeeds.MAX_SPEED));

        // Quarter Speed
        driverController.R2().onTrue(Commands.runOnce(() -> m_robotSpeed = RobotSpeeds.QUARTER_SPEED));
        driverController.R2().onFalse(Commands.runOnce(() -> m_robotSpeed = RobotSpeeds.MAX_SPEED));
        // Select left station
        driverController.L1().whileTrue(aimingRoutines.alignWithCoralStation(true));
        // Select right station
        driverController.R1().whileTrue(aimingRoutines.alignWithCoralStation(false));

        // Point wheels with right joystick
        // driverController
        //         .povLeft()
        //         .whileTrue(m_drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(
        //                 new Rotation2d(-driverController.getRightY(), -driverController.getRightX()))));

        // Reset robot orientation
        driverController
                .touchpad()
                .onTrue(m_drivetrain.runOnce(() -> m_drivetrain.setOperatorPerspectiveForward(
                        m_drivetrain.getState().Pose.getRotation())));

        driverController.circle().whileTrue(aimingRoutines.orientToFaceReefWall());
        driverController.triangle().whileTrue(aimingRoutines.alignWithProcessor());
        driverController.cross().whileTrue(aimingRoutines.driveToTree());
        driverController.square().whileTrue(aimingRoutines.driveIntoCage());

        /*
         * Run SysId routines when holding back/start and X/Y.
         * Note that each routine should be run exactly once in a single log.
         * Comment out when finished.
         */
        // driverController
        //         .share()
        //         .and(driverController.povUp())
        //         .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // driverController
        //         .share()
        //         .and(driverController.povDown())
        //         .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // driverController
        //         .options()
        //         .and(driverController.povUp())
        //         .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driverController
        //         .options()
        //         .and(driverController.povDown())
        //         .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this method to define controller input->command mappings.
     * please use <a href="
     * https://www.padcrafter.com/index.php?templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&rightTrigger=Control+Climb+with+joysticks&leftTrigger=Control+both+Climb+with+left+joystick&leftBumper=Raise+catcher+to+coral+station&rightBumper=Catcher+to+trough&aButton=Raise+catcher+and+run+wheels&bButton=Spit+out+coral&xButton=Run+intake+until+algae+is+collected&yButton=Reverse+intake&dpadUp=Move+intake+to+algae+on+a+coral&dpadDown=Move+intake+to+ground&dpadLeft=Remove+algae+from+reef&dpadRight=Intake+to+processor&startButton=Stow+intake&backButton=Stow+catcher&leftStickClick=Manual+catcher+control&rightStickClick=Manual+intake+control
     * ">this controller map</a>
     * to update and view the current controls.
     */
    private void configureOperatorControls() {
        operatorController
                .rightTrigger()
                .whileTrue(m_climb.setPowerCommand(
                        () -> -operatorController.getLeftY(), () -> -operatorController.getRightY()));
        operatorController
                .leftTrigger()
                .whileTrue(m_climb.setPowerCommand(
                        () -> -operatorController.getLeftY(), () -> -operatorController.getLeftY()));

        operatorController.leftStick().toggleOnTrue(m_catcher.setArmPowerCommand(() -> -operatorController.getLeftY()));
        operatorController.rightStick().toggleOnTrue(m_intake.armTestCommand(() -> -operatorController.getRightY()));

        operatorController
                .leftBumper()
                .toggleOnTrue(m_catcher.setArmPositionCommand(CatcherArmPositions.CORAL_STATION));
        operatorController.rightBumper().toggleOnTrue(m_catcher.setArmPositionCommand(CatcherArmPositions.TROUGH));

        operatorController.a().whileTrue(m_catcher.raiseAndRunWheelCommand());
        operatorController.b().whileTrue(m_catcher.wheelBackwardsCommand());

        operatorController.povLeft().whileTrue(m_catcher.wheelBackwardsWhileRaisingArmCommand());
        operatorController.povDown().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.GROUND_PICKUP));
        operatorController.povUp().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.ON_CORAL_PICKUP));
        operatorController.povRight().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.PROCESSOR));

        operatorController.x().whileTrue(m_intake.runWheelsCommand());
        operatorController.y().whileTrue(m_intake.wheelsBackwardsCommand());

        operatorController.back().onTrue(m_catcher.setArmPositionCommand(CatcherArmPositions.STOWED));
        operatorController.start().onTrue(m_intake.setArmPositionCommand(IntakeArmPositions.STOWED));
    }

    /**
     * Registers the <a href="https://pathplanner.dev/pplib-named-commands.html">Named Commands</a> used in PathPlanner.
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand(
                "Lower Catcher Arm", m_catcher.setArmPositionAndEndCommand(CatcherArmPositions.TROUGH));
        NamedCommands.registerCommand(
                "Eject Coral", m_catcher.wheelBackwardsCommand().withTimeout(0.5));
        NamedCommands.registerCommand(
                "Raise Catcher Arm", m_catcher.setArmPositionAndEndCommand(CatcherArmPositions.CORAL_STATION));
        NamedCommands.registerCommand(
                "Intake Coral", m_catcher.runWheelCommand().withTimeout(1));
        NamedCommands.registerCommand("Start Auto Timer", Commands.runOnce(m_autoTimer::resetAndStart));
        NamedCommands.registerCommand("End Auto Timer", Commands.runOnce(m_autoTimer::stopAndPublish));
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
        if (m_robotSpeed == RobotSpeeds.HALF_SPEED) {
            velocityX *= 0.5;
        } else if (m_robotSpeed == RobotSpeeds.QUARTER_SPEED) {
            velocityX *= 0.25;
        }
        return velocityX;
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * Y is defined as left.
     */
    public double getVelocityY() {
        double velocityY = DriveConstants.MAX_LINEAR_SPEED * -driverController.getLeftX();
        if (m_robotSpeed == RobotSpeeds.HALF_SPEED) {
            velocityY *= 0.5;
        } else if (m_robotSpeed == RobotSpeeds.QUARTER_SPEED) {
            velocityY *= 0.25;
        }
        return velocityY;
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * rotation is defined as counterclockwise.
     */
    public double getRotationalRate() {
        double rotationalRate = DriveConstants.MAX_ANGULAR_RATE * -driverController.getRightX();
        if (m_robotSpeed == RobotSpeeds.HALF_SPEED) {
            rotationalRate *= 0.5;
        } else if (m_robotSpeed == RobotSpeeds.QUARTER_SPEED) {
            rotationalRate *= 0.25;
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
        if (m_robotSpeed == RobotSpeeds.HALF_SPEED) {
            deadband *= 0.5;
        } else if (m_robotSpeed == RobotSpeeds.QUARTER_SPEED) {
            deadband *= 0.25;
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
        if (m_robotSpeed == RobotSpeeds.HALF_SPEED) {
            rotationalDeadband *= 0.5;
        } else if (m_robotSpeed == RobotSpeeds.QUARTER_SPEED) {
            rotationalDeadband *= 0.25;
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
