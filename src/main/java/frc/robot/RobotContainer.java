// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorArmPositions;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeArmPositions;
import frc.robot.util.AutoCreator;

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
    private final Elevator m_elevator = new Elevator();
    private final Intake m_intake = new Intake();

    /* Setting up bindings for necessary control of the swerve drive platform.
     */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake()
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
    private RobotSpeeds m_robotSpeed = RobotSpeeds.MAX_SPEED;

    private final DriveTelemetry driveTelemetry = new DriveTelemetry();

    /* Selectors (open up in a dashboard like Elastic) */
    private final SendableChooser<Command> testAutoChooser;
    private final SendableChooser<CageLocation> cageChooser = new SendableChooser<CageLocation>();
    private final AutoCreator autoCreator = new AutoCreator(m_elevator);

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

        testAutoChooser = AutoBuilder.buildAutoChooser();
        testAutoChooser.addOption(
                "Drive Wheel Radius Characterization",
                WheelRadiusCharacterization.characterizationCommand(m_drivetrain));
        testAutoChooser.addOption("Drive to nearest tree", aimingRoutines.driveToTree());
        testAutoChooser.addOption("Drive into cage", aimingRoutines.driveIntoCage());
        SmartDashboard.putData("Select your test auto:", testAutoChooser);

        cageChooser.addOption("Left", CageLocation.LEFT);
        cageChooser.addOption("Middle", CageLocation.MIDDLE);
        cageChooser.addOption("Right", CageLocation.RIGHT);
        SmartDashboard.putData("Select your cage:", cageChooser);

        autoCreator.sendAutoChoosers();
    }

    private void setDefaultCommands() {
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(() -> drive.withVelocityX(getVelocityX())
                .withVelocityY(getVelocityY())
                .withRotationalRate(getRotationalRate())
                .withDeadband(getDeadband())
                .withRotationalDeadband(getRotationalDeadband())));
        m_climb.setDefaultCommand(m_climb.disableMotorsCommand());
        m_elevator.setDefaultCommand(m_elevator.disableMotorsCommand());
        m_intake.setDefaultCommand(m_intake.disableMotorsCommand());
    }

    /**
     * Use this method to define trigger->command mappings that don't involve controller inputs.
     */
    private void configureTriggers() {}

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

        // driverController.circle().whileTrue(aimingRoutines.orientToFaceReefWall());
        driverController.circle().whileTrue(m_drivetrain.applyRequest(() -> brake));
        driverController.triangle().whileTrue(aimingRoutines.alignWithProcessor());
        driverController.cross().whileTrue(aimingRoutines.driveToTree());
        driverController.square().whileTrue(aimingRoutines.driveIntoCage());

        /*
         * Run SysId routines when holding back/start and X/Y.
         * Note that each routine should be run exactly once in a single log.
         * Comment out when finished.
         */
        driverController
                .share()
                .and(driverController.povUp())
                .whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverController
                .share()
                .and(driverController.povDown())
                .whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        driverController
                .options()
                .and(driverController.povUp())
                .whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverController
                .options()
                .and(driverController.povDown())
                .whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
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

        operatorController
                .leftStick()
                .toggleOnTrue(m_elevator.setElevatorPowerCommand(() -> -operatorController.getLeftY()));
        // operatorController.rightStick().toggleOnTrue(m_intake.armTestCommand(() -> -operatorController.getRightY()));
        operatorController
                .rightStick()
                .toggleOnTrue(m_elevator.setArmPowerCommand(() -> -operatorController.getRightY()));

        operatorController.a().whileTrue(m_elevator.runWheelCommand());
        operatorController.b().whileTrue(m_elevator.wheelBackwardsCommand());
        operatorController
                .leftBumper()
                .onTrue(m_elevator.profiledSetArmPositionCommand(ElevatorArmPositions.CORAL_STATION));
        operatorController.rightBumper().onTrue(m_elevator.profiledSetArmPositionCommand(ElevatorArmPositions.STOWED));

        operatorController.povRight().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.PROCESSOR));
        operatorController.povLeft().whileTrue(m_elevator.wheelBackwardsWhileRaisingArmCommand());
        // operatorController.povDown().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.GROUND_PICKUP));
        // operatorController.povUp().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.ON_CORAL_PICKUP));

        // operatorController.x().whileTrue(m_intake.runWheelsCommand());
        // operatorController.y().whileTrue(m_intake.wheelsBackwardsCommand());

        // operatorController.back().onTrue(m_elevator.setArmPositionCommand(CatcherArmPositions.STOWED));
        // operatorController.start().onTrue(m_intake.setArmPositionCommand(IntakeArmPositions.STOWED));

        // operatorController
        //         .back()
        //         .and(operatorController.povUp())
        //         .whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // operatorController
        //         .back()
        //         .and(operatorController.povDown())
        //         .whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // operatorController
        //         .start()
        //         .and(operatorController.povUp())
        //         .whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // operatorController
        //         .start()
        //         .and(operatorController.povDown())
        //         .whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Registers the <a href="https://pathplanner.dev/pplib-named-commands.html">Named Commands</a> used in PathPlanner.
     */
    private void registerNamedCommands() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return testAutoChooser.getSelected();
        return autoCreator.getAutonomousCommand();
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

    public void resetFieldPosition(Pose2d position) {
        m_drivetrain.resetPose(position);
    }

    public void resetRobotRotation(Rotation2d rotation) {
        m_drivetrain.resetRotation(rotation);
    }
}
