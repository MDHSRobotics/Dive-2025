package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPositions;

/**
 * This util class is specific to our PathPlanner path naming scheme for this year.
 */
public class AutoCreator {
    private final SendableChooser<String> m_startPositionChooser = new SendableChooser<String>();
    private final SendableChooser<String> m_treeChooser = new SendableChooser<String>();
    private final SendableChooser<ElevatorPositions> m_levelChooser = new SendableChooser<ElevatorPositions>();
    private final SendableChooser<String> m_coralStationChooser = new SendableChooser<String>();
    private final SendableChooser<String> m_treeChooser2 = new SendableChooser<String>();
    private final SendableChooser<ElevatorPositions> m_levelChooser2 = new SendableChooser<ElevatorPositions>();
    private final SendableChooser<String> m_coralStationChooser2 = new SendableChooser<String>();
    private final SendableChooser<String> m_treeChooser3 = new SendableChooser<String>();
    private final SendableChooser<ElevatorPositions> m_levelChooser3 = new SendableChooser<ElevatorPositions>();

    private final Elevator m_elevator;
    private final AutoTimer m_autoTimer = new AutoTimer();
    private Command m_autoSequence = null;

    public AutoCreator(Elevator elevator) {
        m_elevator = elevator;
    }

    public void sendAutoChoosers() {
        // Starting position
        m_startPositionChooser.addOption("Top", "Top to ");
        m_startPositionChooser.addOption("Middle", "Middle to ");
        m_startPositionChooser.addOption("Bottom", "Bottom to ");
        SmartDashboard.putData("Starting position:", m_startPositionChooser);

        // Options for first coral
        m_treeChooser.addOption("A", "A");
        m_treeChooser.addOption("B", "B");
        m_treeChooser.addOption("C", "C");
        m_treeChooser.addOption("D", "D");
        m_treeChooser.addOption("E", "E");
        m_treeChooser.addOption("F", "F");
        m_treeChooser.addOption("G", "G");
        m_treeChooser.addOption("H", "H");
        m_treeChooser.addOption("I", "I");
        m_treeChooser.addOption("J", "J");
        m_treeChooser.addOption("K", "K");
        m_treeChooser.addOption("L", "L");
        SmartDashboard.putData("Tree #1:", m_treeChooser);

        m_levelChooser.addOption("L1", ElevatorPositions.L1);
        m_levelChooser.addOption("L2", ElevatorPositions.L2);
        m_levelChooser.addOption("L3", ElevatorPositions.L3);
        m_levelChooser.addOption("L4", ElevatorPositions.L4);
        SmartDashboard.putData("Level #1:", m_levelChooser);

        // Options for second coral
        m_coralStationChooser.addOption("End Auto", null);
        m_coralStationChooser.addOption("Top", "Top                   ");
        m_coralStationChooser.addOption("Bottom", "Bottom");
        m_coralStationChooser.onChange(this::createOneCoralAuto);
        SmartDashboard.putData("Coral station #1:", m_coralStationChooser);

        m_treeChooser2.addOption("A", "A");
        m_treeChooser2.addOption("B", "B");
        m_treeChooser2.addOption("C", "C");
        m_treeChooser2.addOption("D", "D");
        m_treeChooser2.addOption("E", "E");
        m_treeChooser2.addOption("F", "F");
        m_treeChooser2.addOption("G", "G");
        m_treeChooser2.addOption("H", "H");
        m_treeChooser2.addOption("I", "I");
        m_treeChooser2.addOption("J", "J");
        m_treeChooser2.addOption("K", "K");
        m_treeChooser2.addOption("L", "L");
        SmartDashboard.putData("Tree #2:", m_treeChooser2);

        m_levelChooser2.addOption("L1", ElevatorPositions.L1);
        m_levelChooser2.addOption("L2", ElevatorPositions.L2);
        m_levelChooser2.addOption("L3", ElevatorPositions.L3);
        m_levelChooser2.addOption("L4", ElevatorPositions.L4);
        SmartDashboard.putData("Level #2:", m_levelChooser2);

        // Options for third coral
        m_coralStationChooser2.addOption("End Auto", null);
        m_coralStationChooser2.addOption("Top", "Top");
        m_coralStationChooser2.addOption("Bottom", "Bottom");
        m_coralStationChooser2.onChange(this::createTwoCoralAuto);
        SmartDashboard.putData("Coral station #2:", m_coralStationChooser2);

        m_treeChooser3.addOption("A", "A");
        m_treeChooser3.addOption("B", "B");
        m_treeChooser3.addOption("C", "C");
        m_treeChooser3.addOption("D", "D");
        m_treeChooser3.addOption("E", "E");
        m_treeChooser3.addOption("F", "F");
        m_treeChooser3.addOption("G", "G");
        m_treeChooser3.addOption("H", "H");
        m_treeChooser3.addOption("I", "I");
        m_treeChooser3.addOption("J", "J");
        m_treeChooser3.addOption("K", "K");
        m_treeChooser3.addOption("L", "L");
        SmartDashboard.putData("Tree #3:", m_treeChooser3);

        m_levelChooser3.addOption("L1", ElevatorPositions.L1);
        m_levelChooser3.addOption("L2", ElevatorPositions.L2);
        m_levelChooser3.addOption("L3", ElevatorPositions.L3);
        m_levelChooser3.addOption("L4", ElevatorPositions.L4);
        m_levelChooser3.onChange(this::createThreeCoralAuto);
        SmartDashboard.putData("Level #3 and Create Auto:", m_levelChooser3);
    }

    private void createOneCoralAuto(String coralStationSelection) {
        if (coralStationSelection == null) {
            try {
                String pathName = m_startPositionChooser.getSelected();
                pathName += m_treeChooser.getSelected();
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                m_autoSequence = Commands.sequence(
                        Commands.parallel(Commands.runOnce(m_autoTimer::resetAndStart), AutoBuilder.followPath(path)),
                        Commands.runOnce(m_autoTimer::stopAndPublish));
                // TODO: Add elevator command to sequence
            } catch (Exception e) {
                DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
                m_autoSequence = Commands.none();
                return;
            }
        }
    }

    private void createTwoCoralAuto(String coralStationSelection) {
        if (coralStationSelection == null) {
            try {
                String pathToFirstTreeName = m_startPositionChooser.getSelected();
                String firstTree = m_treeChooser.getSelected();
                pathToFirstTreeName += firstTree;
                PathPlannerPath pathToFirstTree = PathPlannerPath.fromPathFile(pathToFirstTreeName);

                String pathToCoralStationName = firstTree + " to ";
                String coralStation = m_coralStationChooser.getSelected();
                pathToCoralStationName += coralStation;
                PathPlannerPath pathToCoralStation = PathPlannerPath.fromPathFile(pathToCoralStationName);

                String pathToSecondTreeName = coralStation + " to ";
                pathToSecondTreeName += m_treeChooser2.getSelected();
                PathPlannerPath pathToSecondTree = PathPlannerPath.fromPathFile(pathToSecondTreeName);

                m_autoSequence = Commands.sequence(
                        Commands.parallel(
                                Commands.runOnce(m_autoTimer::resetAndStart), AutoBuilder.followPath(pathToFirstTree)),
                        AutoBuilder.followPath(pathToCoralStation),
                        AutoBuilder.followPath(pathToSecondTree),
                        Commands.runOnce(m_autoTimer::stopAndPublish));
                // TODO: Add elevator command to sequence
            } catch (Exception e) {
                DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
                m_autoSequence = Commands.none();
                return;
            }
        }
        // TODO: finish
    }

    private void createThreeCoralAuto(ElevatorPositions elevatorPosition3) {
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
