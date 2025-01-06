// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {}

    public static class DriverConstants {
        private DriverConstants() {}

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    /**
     * This class contains information about the field, like the positions of game elements and Apriltags.
     * <p>
     * This information comes from <a href="https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf">the FRC Field Drawings.</a>
     */
    public static class FieldConstants {
        private FieldConstants() {}

        /**
         * See page 24 of <a href="https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">the game manual</a> to understand what each letter means.
         * The x and y coordinates were found by importing the Field CAD to Onshape, generating a top-down drawing,
         * and using the dimension tool from the edge of the driver station wall (for x) or the edge of the side polycarbonate wall (for y) to the centermark of the L4 branch.
         * The rotation represents what angle the robot could face to align with the reef wall.
         */
        public static final Pose2d[] BLUE_REEF_BRANCH_POSES = {
            // A
            new Pose2d(Inches.of(145.490), Inches.of(164.968), new Rotation2d()),
            // B
            new Pose2d(Inches.of(145.445), Inches.of(152.030), new Rotation2d()),
            // C
            new Pose2d(Inches.of(155.234), Inches.of(135.152), new Rotation2d(Degrees.of(60))),
            // D
            new Pose2d(Inches.of(166.416), Inches.of(128.645), new Rotation2d(Degrees.of(60))),
            // E
            new Pose2d(Inches.of(185.927), Inches.of(128.684), new Rotation2d(Degrees.of(120))),
            // F
            new Pose2d(Inches.of(197.154), Inches.of(135.113), new Rotation2d(Degrees.of(120))),
            // G
            new Pose2d(Inches.of(206.876), Inches.of(152.030), new Rotation2d(Degrees.of(180))),
            // H
            new Pose2d(Inches.of(206.921), Inches.of(164.968), new Rotation2d(Degrees.of(180))),
            // I
            new Pose2d(Inches.of(197.132), Inches.of(181.846), new Rotation2d(Degrees.of(240))),
            // J
            new Pose2d(Inches.of(185.950), Inches.of(188.354), new Rotation2d(Degrees.of(240))),
            // K
            new Pose2d(Inches.of(166.439), Inches.of(188.315), new Rotation2d(Degrees.of(300))),
            // L
            new Pose2d(Inches.of(155.212), Inches.of(181.885), new Rotation2d(Degrees.of(300)))
        };

        /**
         * See page 24 of <a href="https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">the game manual</a> to understand what each letter means.
         * The x and y coordinates were found by importing the Field CAD to Onshape, generating a top-down drawing,
         * and using the dimension tool from the edge of the driver station wall (for x) or the edge of the side polycarbonate wall (for y) to the centermark of the L4 branch.
         * The rotation represents what angle the robot could face to align with the reef wall.
         */
        public static final Pose2d[] RED_REEF_BRANCH_POSES = {
            // A
            new Pose2d(Inches.of(544.261), Inches.of(152.030), new Rotation2d(Degrees.of(180))),
            // B
            new Pose2d(Inches.of(544.306), Inches.of(164.968), new Rotation2d(Degrees.of(180))),
            // C
            new Pose2d(Inches.of(534.517), Inches.of(181.846), new Rotation2d(Degrees.of(240))),
            // D
            new Pose2d(Inches.of(523.335), Inches.of(188.354), new Rotation2d(Degrees.of(240))),
            // E
            new Pose2d(Inches.of(503.823), Inches.of(188.315), new Rotation2d(Degrees.of(300))),
            // F
            new Pose2d(Inches.of(492.597), Inches.of(181.885), new Rotation2d(Degrees.of(300))),
            // G
            new Pose2d(Inches.of(482.875), Inches.of(164.968), new Rotation2d()),
            // H
            new Pose2d(Inches.of(482.830), Inches.of(152.030), new Rotation2d()),
            // I
            new Pose2d(Inches.of(492.619), Inches.of(135.152), new Rotation2d(Degrees.of(60))),
            // J
            new Pose2d(Inches.of(503.801), Inches.of(128.645), new Rotation2d(Degrees.of(60))),
            // K
            new Pose2d(Inches.of(523.312), Inches.of(128.684), new Rotation2d(Degrees.of(120))),
            // L
            new Pose2d(Inches.of(534.539), Inches.of(135.113), new Rotation2d(Degrees.of(120)))
        };

        // Positions of the Apriltags for logging currently visible vision targets in AdvantageScope.
        // IMPORTANT: Index 0 corresponds to tag id 1. Index 21 corresponds to tag id 22. Basically, index = tag id + 1.
        public static final Pose3d[] APRILTAG_POSES = {
            new Pose3d(
                    Inches.of(657.37),
                    Inches.of(25.80),
                    Inches.of(58.50),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(126))),
            new Pose3d(
                    Inches.of(657.37),
                    Inches.of(291.20),
                    Inches.of(58.50),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(234))),
            new Pose3d(
                    Inches.of(455.15),
                    Inches.of(317.15),
                    Inches.of(51.25),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(270))),
            new Pose3d(
                    Inches.of(365.20),
                    Inches.of(241.64),
                    Inches.of(573.54),
                    new Rotation3d(Degrees.of(30), Degrees.of(0), Degrees.of(0))),
            new Pose3d(
                    Inches.of(365.20),
                    Inches.of(75.39),
                    Inches.of(73.54),
                    new Rotation3d(Degrees.of(30), Degrees.of(0), Degrees.of(0))),
            new Pose3d(
                    Inches.of(530.49),
                    Inches.of(130.17),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(300))),
            new Pose3d(
                    Inches.of(546.87),
                    Inches.of(158.50),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))),
            new Pose3d(
                    Inches.of(530.49),
                    Inches.of(186.83),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(60))),
            new Pose3d(
                    Inches.of(497.77),
                    Inches.of(186.83),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(120))),
            new Pose3d(
                    Inches.of(481.39),
                    Inches.of(158.50),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180))),
            new Pose3d(
                    Inches.of(497.77),
                    Inches.of(130.17),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(240))),
            new Pose3d(
                    Inches.of(33.51),
                    Inches.of(25.80),
                    Inches.of(58.50),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(54))),
            new Pose3d(
                    Inches.of(33.51),
                    Inches.of(291.20),
                    Inches.of(58.50),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(306))),
            new Pose3d(
                    Inches.of(325.68),
                    Inches.of(241.64),
                    Inches.of(73.54),
                    new Rotation3d(Degrees.of(30), Degrees.of(0), Degrees.of(180))),
            new Pose3d(
                    Inches.of(325.68),
                    Inches.of(75.39),
                    Inches.of(73.54),
                    new Rotation3d(Degrees.of(30), Degrees.of(0), Degrees.of(180))),
            new Pose3d(
                    Inches.of(235.73),
                    Inches.of(-0.15),
                    Inches.of(51.25),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(90))),
            new Pose3d(
                    Inches.of(160.39),
                    Inches.of(130.17),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(240))),
            new Pose3d(
                    Inches.of(144.00),
                    Inches.of(158.50),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180))),
            new Pose3d(
                    Inches.of(160.39),
                    Inches.of(186.83),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(120))),
            new Pose3d(
                    Inches.of(193.10),
                    Inches.of(186.83),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(60))),
            new Pose3d(
                    Inches.of(209.49),
                    Inches.of(158.50),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))),
            new Pose3d(
                    Inches.of(193.10),
                    Inches.of(130.17),
                    Inches.of(12.13),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(300)))
        };
    }
}
