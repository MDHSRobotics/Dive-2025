// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.drive.ProfiledFieldCentricFacingPosition;
import frc.robot.subsystems.drive.TunerConstants;

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
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_LINEAR_SPEED * 0.1)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * 0.1);

    private final ProfiledFieldCentricFacingAngle driveFacingAngle = new ProfiledFieldCentricFacingAngle(
                    DriveConstants.ANGULAR_MOTION_CONSTRAINTS)
            .withPIDGains(DriveConstants.K_P, 0, DriveConstants.K_D)
            .withDeadband(DriveConstants.MAX_LINEAR_SPEED * 0.1)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * 0.1);

    private final ProfiledFieldCentricFacingPosition driveFacingPosition = new ProfiledFieldCentricFacingPosition(
                    DriveConstants.ANGULAR_MOTION_CONSTRAINTS)
            .withPIDGains(DriveConstants.K_P, 0, DriveConstants.K_D)
            .withDeadband(DriveConstants.MAX_LINEAR_SPEED * 0.1)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * 0.1);

    private final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt();

    /* Controllers */
    private final CommandPS4Controller driverController =
            new CommandPS4Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    /* NetworkTables Logging */
    private final DriveTelemetry driveTelemetry = new DriveTelemetry();
    private final NetworkTable driverInfoTable =
            NetworkTableInstance.getDefault().getTable("Driver Info");
    /**
     * See page 24 of <a href="https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">the game manual</a> to understand what each letter means.
     */
    private final BooleanPublisher[] selectedTreeIndicators = {
        driverInfoTable.getBooleanTopic("Tree A").publish(),
        driverInfoTable.getBooleanTopic("Tree B").publish(),
        driverInfoTable.getBooleanTopic("Tree C").publish(),
        driverInfoTable.getBooleanTopic("Tree D").publish(),
        driverInfoTable.getBooleanTopic("Tree E").publish(),
        driverInfoTable.getBooleanTopic("Tree F").publish(),
        driverInfoTable.getBooleanTopic("Tree G").publish(),
        driverInfoTable.getBooleanTopic("Tree H").publish(),
        driverInfoTable.getBooleanTopic("Tree I").publish(),
        driverInfoTable.getBooleanTopic("Tree J").publish(),
        driverInfoTable.getBooleanTopic("Tree K").publish(),
        driverInfoTable.getBooleanTopic("Tree L").publish()
    };

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
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(getVelocityX())
                .withVelocityY(getVelocityY())
                .withRotationalRate(getRotationalRate())));
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
     * please use <a href="https://www.padcrafter.com/?templates=Driver+Controller&plat=1&leftStick=Drive&aButton=Point+wheels+with+left+joystick&xButton=&yButton=&leftBumper=Reset+robot+orientation">this controller map</a>
     * to update and view the current controls.
     */
    private void configureDriverControls() {
        driverController
                .cross()
                .whileTrue(drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        driverController.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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
        return DriveConstants.MAX_LINEAR_SPEED * -driverController.getLeftY();
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * Y is defined as left.
     */
    public double getVelocityY() {
        return DriveConstants.MAX_LINEAR_SPEED * -driverController.getLeftX();
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * rotation is defined as counterclockwise.
     */
    public double getRotationalRate() {
        return DriveConstants.MAX_ANGULAR_RATE * -driverController.getRightX();
    }
}
