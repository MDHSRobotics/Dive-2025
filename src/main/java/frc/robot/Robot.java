// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.Elastic;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.urcl.URCL;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    private boolean m_hasAppliedRobotRotation;
    private DoublePublisher m_matchTimePub;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Silence joystick warnings because they get in the way of other warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        m_hasAppliedRobotRotation = false;

        m_matchTimePub =
                NetworkTableInstance.getDefault().getDoubleTopic("Match Time").publish();

        // Create the webserver for accessing Elastic's saved layout across computers
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        Elastic.selectTab("Autonomous");

        // Configure limelight position
        LimelightHelpers.setCameraPose_RobotSpace(
                VisionConstants.FRONT_LIMELIGHT_NAME,
                VisionConstants.FRONT_LIMELIGHT_FORWARD_OFFSET,
                0,
                VisionConstants.FRONT_LIMELIGHT_UP_OFFSET,
                0,
                0,
                0);
        LimelightHelpers.setCameraPose_RobotSpace(
                VisionConstants.BACK_LIMELIGHT_NAME,
                VisionConstants.BACK_LIMELIGHT_FORWARD_OFFSET,
                0,
                VisionConstants.BACK_LIMELIGHT_UP_OFFSET,
                0,
                0,
                VisionConstants.BACK_LIMELIGHT_YAW);

        // Ensure that the PathPlanner GUI displays the robot's actual config.
        // If there are differences, they will be reported in SmartDashboard.
        // DriveConstants.PATHPLANNER_CONFIG.hasValidConfig();

        // Turn the LEDs red
        // LEDs.candle.setLEDs(255, 0, 0, 0, LEDConstants.LED_STRIP_START, LEDConstants.LED_STRIP_COUNT);

        SignalLogger.setPath("/media/sda1/logs/");
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);
        URCL.start(Constants.REV_CAN_ID_ALIASES);
        SignalLogger.start();

        FollowPathCommand.warmupCommand().schedule();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        m_matchTimePub.set(DriverStation.getMatchTime());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        // Turn the LEDs white to indicate the robot is connected to Driver Station/FMS
        // LEDs.candle.setLEDs(255, 255, 255, 0, LEDConstants.LED_STRIP_START, LEDConstants.LED_STRIP_COUNT);
    }

    @Override
    public void disabledPeriodic() {
        /* You need to update the robot rotation before PathPlanner does
        because time between updating the rotation and the limelight receiving it
        causes the limelight to send incorrect pose estimates for a split second at the start of the match.
        */
        if (!m_hasAppliedRobotRotation) {
            Alliance alliance = DriverStation.getAlliance().orElse(null);
            if (alliance == Alliance.Blue) {
                m_robotContainer.resetRobotRotation(Rotation2d.k180deg);
                m_hasAppliedRobotRotation = true;
            } else if (alliance == Alliance.Red) {
                m_robotContainer.resetRobotRotation(Rotation2d.kZero);
                m_hasAppliedRobotRotation = true;
            }
        }
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            System.out.println("Starting auto: " + m_autonomousCommand.getName());
            m_autonomousCommand.schedule();
            // LEDs.candle.setLEDs(0, 0, 255, 0, LEDConstants.LED_STRIP_START, LEDConstants.LED_STRIP_COUNT);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        Elastic.selectTab("Teleoperated");
        // LEDs.candle.setLEDs(0, 255, 0, 0, LEDConstants.LED_STRIP_START, LEDConstants.LED_STRIP_COUNT);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        m_robotContainer.resetFieldPosition(new Pose2d(Meters.of(10), Meters.of(5), Rotation2d.fromDegrees(180)));
        // LEDs.candle.setLEDs(255, 0, 255, 0, LEDConstants.LED_STRIP_START, LEDConstants.LED_STRIP_COUNT);
        // SignalLogger.start();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        // SignalLogger.stop();
        // DataLogManager.stop();
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
