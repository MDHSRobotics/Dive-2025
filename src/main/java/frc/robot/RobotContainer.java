// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandPS4Controller driverController =
            new CommandPS4Controller(DriverConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(DriverConstants.OPERATOR_CONTROLLER_PORT);

    /* Robot States */
    private int selectedBranchIndex = 0;

    /* NetworkTables Logging */
    private final NetworkTable driverInfoTable =
            NetworkTableInstance.getDefault().getTable("Driver Info");
    /**
     * See page 24 of <a href="https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">the game manual</a> to understand what each letter means.
     */
    private final BooleanPublisher[] selectedBranchIndicators = {
        driverInfoTable.getBooleanTopic("Branch A").publish(),
        driverInfoTable.getBooleanTopic("Branch B").publish(),
        driverInfoTable.getBooleanTopic("Branch C").publish(),
        driverInfoTable.getBooleanTopic("Branch D").publish(),
        driverInfoTable.getBooleanTopic("Branch E").publish(),
        driverInfoTable.getBooleanTopic("Branch F").publish(),
        driverInfoTable.getBooleanTopic("Branch G").publish(),
        driverInfoTable.getBooleanTopic("Branch H").publish(),
        driverInfoTable.getBooleanTopic("Branch I").publish(),
        driverInfoTable.getBooleanTopic("Branch J").publish(),
        driverInfoTable.getBooleanTopic("Branch K").publish(),
        driverInfoTable.getBooleanTopic("Branch L").publish()
    };

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Change the reef wall you want to face counterclockwise.
        driverController.povLeft().onTrue(new InstantCommand(this::selectNextTwoBranches, new Subsystem[0]));

        // Change the reef wall you want to face clockwise.
        driverController.povRight().onTrue(new InstantCommand(this::selectPreviousTwoBranches, new Subsystem[0]));

        // Change the branch you want to face counterclockwise.
        driverController.povUp().onTrue(new InstantCommand(this::selectNextBranch, new Subsystem[0]));

        // Change the branch you want to face clockwise.
        driverController.povDown().onTrue(new InstantCommand(this::selectPreviousBranch, new Subsystem[0]));

        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        operatorController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(exampleSubsystem);
    }

    public void selectNextBranch() {
        selectedBranchIndex += 1;
        // Wrap the index
        if (selectedBranchIndex > 11) {
            selectedBranchIndex -= 12;
        }

        // Update dashboard info
        for (BooleanPublisher branch : selectedBranchIndicators) {
            branch.set(false);
        }
        // Since the driver wants to face a specific branch, only light up that one branch.
        selectedBranchIndicators[selectedBranchIndex].set(true);
    }

    public void selectPreviousBranch() {
        selectedBranchIndex -= 1;
        // Wrap the index
        if (selectedBranchIndex < 0) {
            selectedBranchIndex += 12;
        }

        // Update dashboard info
        for (BooleanPublisher branch : selectedBranchIndicators) {
            branch.set(false);
        }
        // Since the driver wants to face a specific branch, only light up that one branch.
        selectedBranchIndicators[selectedBranchIndex].set(true);
    }

    public void selectNextTwoBranches() {
        selectedBranchIndex = selectedBranchIndex + 2;
        // Wrap the index
        if (selectedBranchIndex > 11) {
            selectedBranchIndex -= 12;
        }

        // Update dashboard info
        for (BooleanPublisher branch : selectedBranchIndicators) {
            branch.set(false);
        }
        // Handle the case where the index may be odd
        if (selectedBranchIndex % 2 != 0) {
            selectedBranchIndex -= 1;
        }
        // Since the driver wants to face a side of the reef wall, they may be trying to score on either of the two
        // branches.
        selectedBranchIndicators[selectedBranchIndex].set(true);
        selectedBranchIndicators[selectedBranchIndex + 1].set(true);
    }

    public void selectPreviousTwoBranches() {
        selectedBranchIndex -= 2;
        // Wrap the index
        if (selectedBranchIndex < 0) {
            selectedBranchIndex += 12;
        }

        // Update dashboard info
        for (BooleanPublisher branch : selectedBranchIndicators) {
            branch.set(false);
        }
        // Handle the case where the index may be odd
        if (selectedBranchIndex % 2 != 0) {
            selectedBranchIndex -= 1;
        }
        // Since the driver wants to face a side of the reef wall, they may be trying to score on either of the two
        // branches.
        selectedBranchIndicators[selectedBranchIndex].set(true);
        selectedBranchIndicators[selectedBranchIndex + 1].set(true);
    }

    public Pose2d getSelectedBranch() {
        Alliance alliance = DriverStation.getAlliance().orElse(null);
        if (alliance == Alliance.Blue) {
            return FieldConstants.BLUE_REEF_BRANCH_POSES[selectedBranchIndex];
        } else if (alliance == Alliance.Red) {
            return FieldConstants.RED_REEF_BRANCH_POSES[selectedBranchIndex];
        }
        DriverStation.reportError("Could not get alliance, please fix this!", true);
        return FieldConstants.BLUE_REEF_BRANCH_POSES[selectedBranchIndex];
    }
}
