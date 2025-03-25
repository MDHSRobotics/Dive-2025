package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class AutoCreator {
    private final SendableChooser<String> m_startPositionChooser = new SendableChooser<String>();
    private final SendableChooser<String> m_reefSideChooser = new SendableChooser<String>();
    private final SendableChooser<String> m_treeChooser = new SendableChooser<String>();
    private final SendableChooser<Double> m_levelChooser = new SendableChooser<Double>();
    private final SendableChooser<String> m_coralStationChooser = new SendableChooser<String>();
    private final SendableChooser<String> m_reefSideChooser2 = new SendableChooser<String>();
    private final SendableChooser<String> m_treeChooser2 = new SendableChooser<String>();
    private final SendableChooser<Double> m_levelChooser2 = new SendableChooser<Double>();
    private final SendableChooser<String> m_coralStationChooser2 = new SendableChooser<String>();
    private final SendableChooser<String> m_reefSideChooser3 = new SendableChooser<String>();
    private final SendableChooser<String> m_treeChooser3 = new SendableChooser<String>();
    private final SendableChooser<Double> m_levelChooser3 = new SendableChooser<Double>();

    private final Elevator m_elevator;
    private Command m_autoSequence = null;

    public AutoCreator(Elevator elevator) {
        m_elevator = elevator;
    }

    public void sendAutoChoosers() {
        // Starting position
        m_startPositionChooser.addOption("Top", "Top to ");
        m_startPositionChooser.addOption("Middle", "Middle to");
        m_startPositionChooser.addOption("Bottom", "Bottom to");
        SmartDashboard.putData("Starting position:", m_startPositionChooser);

        // Options for first coral
        m_reefSideChooser.addOption("1", "1");
        m_reefSideChooser.addOption("2", "2");
        m_reefSideChooser.addOption("3", "3");
        m_reefSideChooser.addOption("4", "4");
        m_reefSideChooser.addOption("5", "5");
        m_reefSideChooser.addOption("6", "6");
        SmartDashboard.putData("Reef side #1:", m_reefSideChooser);

        m_treeChooser.addOption("Left", "(Left Tree)");
        m_treeChooser.addOption("Right", "(Right Tree)");
        SmartDashboard.putData("Tree side #1:", m_treeChooser);

        m_levelChooser.addOption("L1", 1.0);
        m_levelChooser.addOption("L2", 2.0);
        m_levelChooser.addOption("L3", 3.0);
        m_levelChooser.addOption("L4", 4.0);
        SmartDashboard.putData("Level #1:", m_levelChooser);

        // Options for second coral
        m_coralStationChooser.addOption("Top", "Top");
        m_coralStationChooser.addOption("Bottom", "Bottom");
        SmartDashboard.putData("Coral station #1:", m_coralStationChooser);

        m_reefSideChooser2.addOption("1", "1");
        m_reefSideChooser2.addOption("2", "2");
        m_reefSideChooser2.addOption("3", "3");
        m_reefSideChooser2.addOption("4", "4");
        m_reefSideChooser2.addOption("5", "5");
        m_reefSideChooser2.addOption("6", "6");
        SmartDashboard.putData("Reef side #2:", m_reefSideChooser2);

        m_treeChooser2.addOption("Left", "(Left Tree)");
        m_treeChooser2.addOption("Right", "(Right Tree)");
        SmartDashboard.putData("Tree side #2:", m_treeChooser2);

        m_levelChooser2.addOption("L1", 1.0);
        m_levelChooser2.addOption("L2", 2.0);
        m_levelChooser2.addOption("L3", 3.0);
        m_levelChooser2.addOption("L4", 4.0);
        SmartDashboard.putData("Level #2:", m_levelChooser2);

        // Options for third coral
        m_coralStationChooser2.addOption("Top", "Top");
        m_coralStationChooser2.addOption("Bottom", "Bottom");
        SmartDashboard.putData("Coral station #2:", m_coralStationChooser2);

        m_reefSideChooser3.addOption("1", "1");
        m_reefSideChooser3.addOption("2", "2");
        m_reefSideChooser3.addOption("3", "3");
        m_reefSideChooser3.addOption("4", "4");
        m_reefSideChooser3.addOption("5", "5");
        m_reefSideChooser3.addOption("6", "6");
        SmartDashboard.putData("Reef side #3:", m_reefSideChooser3);

        m_treeChooser3.addOption("Left", "(Left Tree)");
        m_treeChooser3.addOption("Right", "(Right Tree)");
        SmartDashboard.putData("Tree side #3:", m_treeChooser3);

        m_levelChooser3.addOption("No second or third coral", null);
        m_levelChooser3.addOption("L1", 1.0);
        m_levelChooser3.addOption("L2", 2.0);
        m_levelChooser3.addOption("L3", 3.0);
        m_levelChooser3.addOption("L4", 4.0);
        m_levelChooser3.onChange(this::acceptFinalOptionAndCreateAuto);
        SmartDashboard.putData("Level #3 and Create Auto:", m_levelChooser3);
    }

    private void acceptFinalOptionAndCreateAuto(Double levelRotations) {
        // TODO: finish
    }

    public Command getAutonomousCommand() {
        if (m_autoSequence != null) {
            return m_autoSequence;
        } else {
            return Commands.none();
        }
    }
}
