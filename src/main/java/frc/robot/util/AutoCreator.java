package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorArmPositions;
import frc.robot.subsystems.elevator.Elevator.ElevatorPositions;
import java.util.function.Consumer;

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

    private final Consumer<Pose2d> m_odometryResetter;
    private final Elevator m_elevator;
    private final AutoTimer m_autoTimer = new AutoTimer();
    private Command m_autoSequence = null;

    public AutoCreator(Consumer<Pose2d> odometryResetter, Elevator elevator) {
        m_odometryResetter = odometryResetter;
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

        m_levelChooser.addOption("L1", ElevatorPositions.STOWED);
        m_levelChooser.addOption("L2", ElevatorPositions.L2);
        m_levelChooser.addOption("L3", ElevatorPositions.L3);
        SmartDashboard.putData("Level #1:", m_levelChooser);

        // Options for second coral
        m_coralStationChooser.addOption("End Auto", null);
        m_coralStationChooser.addOption("Top", "Top Station");
        m_coralStationChooser.addOption("Bottom", "Bottom Station");
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

        m_levelChooser2.addOption("L1", ElevatorPositions.STOWED);
        m_levelChooser2.addOption("L2", ElevatorPositions.L2);
        m_levelChooser2.addOption("L3", ElevatorPositions.L3);
        SmartDashboard.putData("Level #2:", m_levelChooser2);

        // Options for third coral
        m_coralStationChooser2.addOption("End Auto", null);
        m_coralStationChooser2.addOption("Top", "Top Station");
        m_coralStationChooser2.addOption("Bottom", "Bottom Station");
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

        m_levelChooser3.addOption("L1", ElevatorPositions.STOWED);
        m_levelChooser3.addOption("L2", ElevatorPositions.L2);
        m_levelChooser3.addOption("L3", ElevatorPositions.L3);
        m_levelChooser3.onChange(this::createThreeCoralAuto);
        SmartDashboard.putData("Level #3 and Create Auto:", m_levelChooser3);
    }

    private void createOneCoralAuto(String coralStationSelection) {
        if (coralStationSelection == null) {
            try {
                String pathName = m_startPositionChooser.getSelected();
                pathName += m_treeChooser.getSelected();
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                ElevatorPositions elevatorPosition = m_levelChooser.getSelected();
                ElevatorArmPositions armPosition;
                if (elevatorPosition == ElevatorPositions.L2 || elevatorPosition == ElevatorPositions.L3) {
                    armPosition = ElevatorArmPositions.L_2_AND_3;
                } else {
                    armPosition = ElevatorArmPositions.L1;
                }
                m_autoSequence = Commands.sequence(
                        resetOdometryCommand(path.getStartingHolonomicPose().orElseThrow()),
                        Commands.waitSeconds(7.5),
                        Commands.deadline(
                                AutoBuilder.followPath(path),
                                Commands.runOnce(m_autoTimer::resetAndStart),
                                m_elevator.setElevatorAndArmPositionCommand(elevatorPosition, armPosition)),
                        m_elevator.ejectCoralCommand().withTimeout(0.25),
                        Commands.runOnce(m_autoTimer::stopAndPublish));
            } catch (Exception e) {
                DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
                System.out.println("Failed to schedule auto path" + e.getMessage() + e.getStackTrace());

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
                ElevatorPositions firstElevatorPosition = m_levelChooser.getSelected();
                ElevatorArmPositions firstArmPosition;
                if (firstElevatorPosition == ElevatorPositions.L2 || firstElevatorPosition == ElevatorPositions.L3) {
                    firstArmPosition = ElevatorArmPositions.L_2_AND_3;
                } else {
                    firstArmPosition = ElevatorArmPositions.L1;
                }

                String pathToCoralStationName = firstTree + " to ";
                String coralStation = m_coralStationChooser.getSelected();
                pathToCoralStationName += coralStation;
                PathPlannerPath pathToCoralStation = PathPlannerPath.fromPathFile(pathToCoralStationName);

                String pathToSecondTreeName = coralStation + " to ";
                pathToSecondTreeName += m_treeChooser2.getSelected();
                PathPlannerPath pathToSecondTree = PathPlannerPath.fromPathFile(pathToSecondTreeName);
                ElevatorPositions secondElevatorPosition = m_levelChooser2.getSelected();
                ElevatorArmPositions secondArmPosition;
                if (secondElevatorPosition == ElevatorPositions.L2 || secondElevatorPosition == ElevatorPositions.L3) {
                    secondArmPosition = ElevatorArmPositions.L_2_AND_3;
                } else {
                    secondArmPosition = ElevatorArmPositions.L1;
                }

                m_autoSequence = Commands.sequence(
                        resetOdometryCommand(
                                pathToFirstTree.getStartingHolonomicPose().orElseThrow()),
                        Commands.deadline(
                                AutoBuilder.followPath(pathToFirstTree),
                                Commands.runOnce(m_autoTimer::resetAndStart),
                                m_elevator.setElevatorAndArmPositionCommand(firstElevatorPosition, firstArmPosition)),
                        m_elevator.ejectCoralCommand().withTimeout(0.25),
                        Commands.deadline(
                                AutoBuilder.followPath(pathToCoralStation),
                                m_elevator.setElevatorAndArmPositionCommand(
                                        ElevatorPositions.STOWED, ElevatorArmPositions.CORAL_STATION)),
                        m_elevator.intakeCoralCommand().withTimeout(0.75),
                        Commands.deadline(
                                AutoBuilder.followPath(pathToSecondTree),
                                Commands.waitSeconds(1)
                                        .andThen(m_elevator.setElevatorAndArmPositionCommand(
                                                secondElevatorPosition, secondArmPosition))),
                        m_elevator.ejectCoralCommand().withTimeout(0.25),
                        Commands.runOnce(m_autoTimer::stopAndPublish));
            } catch (Exception e) {
                DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
                System.out.println("Failed to schedule auto path" + e.getMessage() + e.getStackTrace());

                m_autoSequence = Commands.none();
                return;
            }
        }
    }

    private void createThreeCoralAuto(ElevatorPositions elevatorPosition3) {
        try {
            String pathToFirstTreeName = m_startPositionChooser.getSelected();
            String firstTree = m_treeChooser.getSelected();
            pathToFirstTreeName += firstTree;
            PathPlannerPath pathToFirstTree = PathPlannerPath.fromPathFile(pathToFirstTreeName);
            ElevatorPositions firstElevatorPosition = m_levelChooser.getSelected();
            ElevatorArmPositions firstArmPosition;
            if (firstElevatorPosition == ElevatorPositions.L2 || firstElevatorPosition == ElevatorPositions.L3) {
                firstArmPosition = ElevatorArmPositions.L_2_AND_3;
            } else {
                firstArmPosition = ElevatorArmPositions.L1;
            }

            String pathToCoralStationName = firstTree + " to ";
            String firstCoralStation = m_coralStationChooser.getSelected();
            pathToCoralStationName += firstCoralStation;
            PathPlannerPath pathToFirstCoralStation = PathPlannerPath.fromPathFile(pathToCoralStationName);

            String pathToSecondTreeName = firstCoralStation + " to ";
            String secondTree = m_treeChooser2.getSelected();
            pathToSecondTreeName += secondTree;
            PathPlannerPath pathToSecondTree = PathPlannerPath.fromPathFile(pathToSecondTreeName);
            ElevatorPositions secondElevatorPosition = m_levelChooser2.getSelected();
            ElevatorArmPositions secondArmPosition;
            if (secondElevatorPosition == ElevatorPositions.L2 || secondElevatorPosition == ElevatorPositions.L3) {
                secondArmPosition = ElevatorArmPositions.L_2_AND_3;
            } else {
                secondArmPosition = ElevatorArmPositions.L1;
            }

            String pathToSecondCoralStationName = secondTree + " to ";
            String secondCoralStation = m_coralStationChooser2.getSelected();
            pathToSecondCoralStationName += secondCoralStation;
            PathPlannerPath pathToSecondCoralStation = PathPlannerPath.fromPathFile(pathToSecondCoralStationName);

            String pathToThirdTreeName = secondCoralStation + " to ";
            pathToThirdTreeName += m_treeChooser3.getSelected();
            PathPlannerPath pathToThirdTree = PathPlannerPath.fromPathFile(pathToThirdTreeName);
            ElevatorPositions thirdElevatorPosition = m_levelChooser3.getSelected();
            ElevatorArmPositions thirdArmPosition;
            if (thirdElevatorPosition == ElevatorPositions.L2 || thirdElevatorPosition == ElevatorPositions.L3) {
                thirdArmPosition = ElevatorArmPositions.L_2_AND_3;
            } else {
                thirdArmPosition = ElevatorArmPositions.L1;
            }

            m_autoSequence = Commands.sequence(
                    resetOdometryCommand(
                            pathToFirstTree.getStartingHolonomicPose().orElseThrow()),
                    Commands.deadline(
                            AutoBuilder.followPath(pathToFirstTree),
                            Commands.runOnce(m_autoTimer::resetAndStart),
                            m_elevator.setElevatorAndArmPositionCommand(firstElevatorPosition, firstArmPosition)),
                    m_elevator.ejectCoralCommand().withTimeout(0.25),
                    Commands.deadline(
                            AutoBuilder.followPath(pathToFirstCoralStation),
                            m_elevator.setElevatorAndArmPositionCommand(
                                    ElevatorPositions.STOWED, ElevatorArmPositions.CORAL_STATION)),
                    m_elevator.intakeCoralCommand().withTimeout(0.75),
                    Commands.deadline(
                            AutoBuilder.followPath(pathToSecondTree),
                            Commands.waitSeconds(1)
                                    .andThen(m_elevator.setElevatorAndArmPositionCommand(
                                            secondElevatorPosition, secondArmPosition))),
                    m_elevator.ejectCoralCommand().withTimeout(0.25),
                    Commands.deadline(
                            AutoBuilder.followPath(pathToSecondCoralStation),
                            m_elevator.setElevatorAndArmPositionCommand(
                                    ElevatorPositions.STOWED, ElevatorArmPositions.CORAL_STATION)),
                    m_elevator.intakeCoralCommand().withTimeout(0.75),
                    Commands.deadline(
                            AutoBuilder.followPath(pathToThirdTree),
                            Commands.waitSeconds(1)
                                    .andThen(m_elevator.setElevatorAndArmPositionCommand(
                                            thirdElevatorPosition, thirdArmPosition))),
                    m_elevator.ejectCoralCommand().withTimeout(0.25),
                    Commands.runOnce(m_autoTimer::stopAndPublish));
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
            System.out.println("Failed to schedule auto path" + e.getMessage() + e.getStackTrace());
            m_autoSequence = Commands.none();
            return;
        }
    }

    public Command resetOdometryCommand(Pose2d startingPose) {
        return Commands.runOnce(() -> {
            Pose2d newStartingPose = startingPose;
            if (DriverStation.getAlliance().orElseThrow() == Alliance.Red) {
                newStartingPose = FlippingUtil.flipFieldPose(startingPose);
            }
            m_odometryResetter.accept(newStartingPose);
        });
    }

    public Command getAutonomousCommand() {
        return m_autoSequence;
    }
}
