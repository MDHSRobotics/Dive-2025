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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.AimingRoutines;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.drive.requests.DriveWithSetpointGeneration;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorArmPositions;
import frc.robot.subsystems.elevator.Elevator.ElevatorPositions;
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
    // The robot's subsystems and commands are defined here...
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final Climb m_climb = new Climb();
    private final Elevator m_elevator = new Elevator();
    private final Intake m_intake = new Intake();
    private final LED m_led = new LED();

    /* Setting up bindings for necessary control of the swerve drive platform.
     */
    private final DriveWithSetpointGeneration m_drive = new DriveWithSetpointGeneration(
                    DriveConstants.SWERVE_SETPOINT_GENERATOR, Constants.UPDATE_PERIOD)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    /* Controllers */
    private final CommandPS4Controller m_driverController =
            new CommandPS4Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController m_operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    /* Robot States */
    /** Current driving speed percentage from 0.0 to 1.0. */
    private double m_robotSpeed = 1.0;

    // Auto alignment begins when the operator pushes the right stick at least 80% in any direction.
    private final Trigger m_autoAlignmentRequested =
            new Trigger(() -> Math.hypot(m_operatorController.getRightX(), -m_operatorController.getRightY()) > 0.8);

    // Coral Detection
    private final Trigger m_coralDetector = new Trigger(() -> m_elevator.detectGamePiece());

    // Match Timer
    private final Trigger m_matchTime = new Trigger(() -> Timer.getMatchTime() < 20);

    private final DriveTelemetry m_driveTelemetry = new DriveTelemetry();

    /* Selectors (open up in a dashboard like Elastic) */
    private final SendableChooser<Command> m_testAutoChooser;
    private final AutoCreator m_autoCreator = new AutoCreator(this::resetFieldPosition, m_elevator);

    private final AimingRoutines m_aimingRoutines = new AimingRoutines(
            m_drivetrain,
            this::getVelocityX,
            this::getVelocityY,
            this::getDeadband,
            m_operatorController::getLeftY,
            m_operatorController::getLeftX,
            m_operatorController::getRightY,
            m_operatorController::getRightX);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        registerNamedCommands();
        setDefaultCommands();
        configureTriggers();
        configureDriverControls();
        configureOperatorControls();

        m_drivetrain.registerTelemetry(m_driveTelemetry::telemeterize);

        m_testAutoChooser = AutoBuilder.buildAutoChooser();
        m_testAutoChooser.addOption("Align with Coral Station", m_aimingRoutines.alignWithCoralStation(true));
        m_testAutoChooser.addOption("Align with Processor", m_aimingRoutines.alignWithProcessor());
        m_testAutoChooser.addOption("Orient to Face Reef Wall", m_aimingRoutines.orientToFaceReefWall());
        m_testAutoChooser.addOption("Drive to Nearest Tree", m_aimingRoutines.driveToNearestTree());
        // testAutoChooser.addOption(
        //         "Drive Wheel Radius Characterization",
        //         WheelRadiusCharacterization.characterizationCommand(m_drivetrain));
        SmartDashboard.putData("Select your test auto:", m_testAutoChooser);

        m_autoCreator.sendAutoChoosers();
    }

    private void setDefaultCommands() {
        m_drivetrain.setDefaultCommand(m_drivetrain.applyResettableRequest(() -> m_drive.withVelocityX(getVelocityX())
                .withVelocityY(getVelocityY())
                .withRotationalRate(getRotationalRate())
                .withDeadband(getDeadband())
                .withRotationalDeadband(getRotationalDeadband())));
        m_climb.setDefaultCommand(m_climb.disableMotorsCommand());
        m_elevator.setDefaultCommand(m_elevator.setElevatorAndArmPositionCommand(
                ElevatorPositions.STOWED, ElevatorArmPositions.CURRENT_POSITION));

        m_intake.setDefaultCommand(m_intake.disableMotorsCommand());
        m_led.setDefaultCommand(new RunCommand(() -> m_led.setRainbowAnimation(), m_led));
    }

    /**
     * Use this method to define trigger->command mappings that don't involve controller inputs.
     */
    private void configureTriggers() {
        m_autoAlignmentRequested.and(() -> m_elevator.getTagID() != -1).onTrue(m_aimingRoutines.driveToTree());

        m_coralDetector
                .debounce(0.2)
                // .and(m_operatorController.leftBumper())
                .onTrue(new RunCommand(() -> m_led.setTwinkleAnimation(), m_led).withTimeout(3));
    }

    /**
     * Use this method to define controller input->command mappings.
     * please use <a href="
     * https://www.padcrafter.com/?templates=Driver+Controller&plat=1&leftStick=Drive&aButton=Lock+wheels&xButton=Re-enable+manual+driving&yButton=Face+processor&leftBumper=Face+Left+Coral+Station&backButton=Reset+robot+orientation&rightBumper=Face+Right+Coral+Station&bButton=Face+reef+wall&leftTrigger=Slow+Mode&rightTrigger=Super+Slow+Mode&dpadLeft=&rightStick=Rotate&dpadDown=Climb&dpadUp=Raise+climb
     * ">this controller map</a>
     * to update and view the current controls.
     */
    /**
     * Updated Driver Controllers
     * https://www.padcrafter.com/?templates=Driver+Controller&plat=1&leftBumper=Face+Left+Coral+Station&rightBumper=Face+Right+Coral+Station&rightTrigger=Slow+Mode+%2825%25%29&dpadUp=Raise+Climb&dpadDown=Lower+Climb&backButton=Reset+Robot+Orientation&yButton=Auto+alignment+to+nearest+coral+station&xButton=Re-enable+manual+driving&bButton=Face+reef+wall&aButton=Lock+Wheels&leftStick=Drive&rightStick=Rotate&leftTrigger=&dpadRight=Remove+Algae+from+reef+%28L3%29&dpadLeft=Remove+Algae+from+reef+%28L1%29
     */
    private void configureDriverControls() {

        // Quarter Speed
        m_driverController.R2().onTrue(Commands.runOnce(() -> m_robotSpeed = 0.25));

        m_driverController.R2().onFalse(Commands.runOnce(() -> m_robotSpeed = 1.0));
        // Facing left coral station
        m_driverController.L1().whileTrue(m_aimingRoutines.alignWithCoralStation(true));
        // Facing right coral station
        m_driverController.R1().whileTrue(m_aimingRoutines.alignWithCoralStation(false));

        // Reset robot orientation
        m_driverController
                .touchpad()
                .onTrue(m_drivetrain.runOnce(() -> m_drivetrain.setOperatorPerspectiveForward(
                        m_drivetrain.getState().Pose.getRotation())));

        // Facing Reef Wall
        m_driverController.circle().whileTrue(m_aimingRoutines.orientToFaceReefWall());
        // Auto alignment to nearest coral station
        m_driverController.triangle().whileTrue(m_aimingRoutines.driveToNearestCoralStation());
        // Lock wheels
        m_driverController.cross().whileTrue(m_drivetrain.applyRequest(() -> m_brake));
        // Press once to begin driving normally again (after auto-alignment)
        m_driverController.square().onTrue(m_drivetrain.applyRequest(() -> m_drive.withVelocityX(getVelocityX())
                .withVelocityY(getVelocityY())
                .withRotationalRate(getRotationalRate())
                .withDeadband(getDeadband())
                .withRotationalDeadband(getRotationalDeadband())));

        // Climb controls
        m_driverController
                .povUp()
                .whileTrue(new ParallelCommandGroup(
                        m_climb.setPowerCommand(() -> 1.0, () -> 1.0),
                        new RunCommand(() -> m_led.setRedGRB(), m_led).withTimeout(5)));
        m_driverController
                .povDown()
                .whileTrue(new ParallelCommandGroup(
                        m_climb.setPowerCommand(() -> -1.0, () -> -1.0),
                        new RunCommand(() -> m_led.setRedGRB(), m_led).withTimeout(5)));
        // Remove algae from reef
        m_driverController.povRight().toggleOnTrue(m_elevator.removeAlgaeFromReefCommand(ElevatorPositions.L3_ALGAE));
        m_driverController.povLeft().toggleOnTrue(m_elevator.removeAlgaeFromReefCommand(ElevatorPositions.STOWED));

        // Raise the Gate for climbing
        m_driverController
                .L2()
                .onTrue(m_elevator
                        .disableWheelMotorsCommand2()
                        .andThen(new ParallelCommandGroup(
                                m_climb.setGatePositionCommand(),
                                m_intake.setArmPositionCommand(IntakeArmPositions.STOWED),
                                m_elevator.setElevatorAndArmPositionCommand(
                                        ElevatorPositions.STOWED, ElevatorArmPositions.STOWED))));
        m_driverController.share().onTrue(m_climb.setGatePowerCommand(() -> -0.5));

        /*
         * Run SysId routines when holding back/start and X/Y.
         * Note that each routine should be run exactly once in a single log.
         * Comment out when finished.
         */
        // m_driverController
        //         .share()
        //         .and(m_driverController.povUp())
        //         .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // m_driverController
        //         .share()
        //         .and(m_driverController.povDown())
        //         .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // m_driverController
        //         .options()
        //         .and(m_driverController.povUp())
        //         .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // m_driverController
        //         .options()
        //         .and(m_driverController.povDown())
        //         .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this method to define controller input->command mappings.
     * please use <a href="
     * https://www.padcrafter.com/index.php?templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&rightTrigger=Elevator+and+arm+to+L2&leftTrigger=Elevator+and+arm+to+L1&leftBumper=Elevator+and+arm+to+coral+station&rightBumper=Elevator+and+arm+to+L3&aButton=Intake+coral&bButton=Eject+coral&xButton=Intake+algae&yButton=Eject+algae&dpadUp=Intake+to+on-coral+algae&dpadDown=Intake+to+ground+algae&dpadLeft=Remove+algae+from+reef&dpadRight=Intake+to+processor&startButton=Stow+intake&backButton=Stow+catcher&leftStickClick=&rightStickClick=&leftStick=Select+reef+side&rightStick=Select+left%2Fright+tree+and+enable+auto+align
     * ">this controller map</a>
     * to update and view the current controls.
     */
    /**
     * Updated Operator Controller
     * https://www.padcrafter.com/?templates=Operator+Controller&yButton=Elevator+and+arm+to+L1&xButton=&bButton=Elevator+and+arm+to+L2&aButton=Elevator+and+arm+to+L3&rightStick=Select+left%2Fright+tree+and+enable+auto+align&dpadUp=Intake+to+processor&dpadRight=Intake+to+algae+on+ice+cream&dpadLeft=&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&leftTrigger=Elevator+and+arm+to+coral+station&leftBumper=Intake+coral&dpadDown=Intake+to+ground&rightTrigger=Eject+Coral&leftStick=Intake+algae&leftStickClick=&startButton=Stow+intake&backButton=Stow+catcher&rightStickClick=eject+algae
     */
    private void configureOperatorControls() {
        // Set Elevator and Arm positions
        // L1
        m_operatorController
                .b()
                .toggleOnTrue(m_elevator.setElevatorAndArmPositionCommand(
                        ElevatorPositions.L2, ElevatorArmPositions.L_2_AND_3));
        m_operatorController
                .a()
                .toggleOnTrue(m_elevator.setElevatorAndArmPositionCommand(
                        ElevatorPositions.L3, ElevatorArmPositions.L_2_AND_3));
        m_operatorController
                .y()
                .toggleOnTrue(
                        m_elevator.setElevatorAndArmPositionCommand(ElevatorPositions.STOWED, ElevatorArmPositions.L1));

        // Intake and Eject corals
        m_operatorController.leftBumper().whileTrue(m_elevator.intakeCoralCommand());
        m_operatorController.rightTrigger().whileTrue(m_elevator.ejectCoralCommand());
        m_operatorController.rightTrigger().onTrue(new InstantCommand(() -> m_led.setRainbowAnimation(), m_led));

        // Elevator and arm to coral station
        m_operatorController
                .leftTrigger()
                .toggleOnTrue(m_elevator.setElevatorAndArmPositionCommand(
                        ElevatorPositions.STOWED, ElevatorArmPositions.CORAL_STATION));

        m_operatorController
                .povRight()
                .toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.ON_CORAL_PICKUP));
        m_operatorController.povDown().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.GROUND_PICKUP));
        m_operatorController.povUp().toggleOnTrue(m_intake.setArmPositionCommand(IntakeArmPositions.PROCESSOR));
        m_operatorController.leftStick().whileTrue(m_intake.runWheelsCommand());
        m_operatorController.rightStick().whileTrue(m_intake.wheelsBackwardsCommand());

        m_operatorController.back().onTrue(m_intake.setArmPositionCommand(IntakeArmPositions.STOWED));
        m_operatorController.start().onTrue(m_elevator.setArmPositionCommand(ElevatorArmPositions.STOWED));

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
        // Uncomment m_autoCreator to run auto
        // return m_testAutoChooser.getSelected();
        return m_autoCreator.getAutonomousCommand();
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * X is defined as forward.
     */
    public double getVelocityX() {
        return -m_driverController.getLeftY() * DriveConstants.MAX_LINEAR_SPEED * m_robotSpeed;
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * Y is defined as left.
     */
    public double getVelocityY() {
        return -m_driverController.getLeftX() * DriveConstants.MAX_LINEAR_SPEED * m_robotSpeed;
    }

    /**
     * According to <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib convention</a>,
     * rotation is defined as counterclockwise.
     */
    public double getRotationalRate() {
        return -m_driverController.getRightX() * DriveConstants.MAX_ANGULAR_RATE * m_robotSpeed;
    }

    /**
     * Deadbands are a percentage of the joystick input.
     * 0.1 means you don't want to move until the joystick is pushed at least 10% in any direction (to prevent drift)
     * @return The linear deadband in meters per second.
     */
    public double getDeadband() {
        return DriveConstants.MAX_LINEAR_SPEED * 0.1 * m_robotSpeed;
    }

    /**
     * Deadbands are a percentage of the joystick input.
     * 0.1 means you don't want to move until the joystick is pushed at least 10% in any direction (to prevent drift)
     * @return The rotational deadband in radians per second.
     */
    public double getRotationalDeadband() {
        return DriveConstants.MAX_ANGULAR_RATE * 0.1 * m_robotSpeed;
    }

    public void resetFieldPosition(Pose2d position) {
        m_drivetrain.resetPose(position);
    }

    public void resetRobotRotation(Rotation2d rotation) {
        m_drivetrain.resetRotation(rotation);
    }
}
