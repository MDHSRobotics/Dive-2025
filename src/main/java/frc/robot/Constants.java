// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.catcher.CatcherConstants;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.Aiming;
import java.util.List;
import java.util.Map;

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

    public static class ControllerConstants {
        private ControllerConstants() {}

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class VisionConstants {
        private VisionConstants() {}

        public static final String FRONT_LIMELIGHT_NAME = "limelight";
        /**
         * Distance from the center of the robot to the front limelight lens in meters.
         */
        public static final double FRONT_LIMELIGHT_FORWARD_OFFSET =
                Inches.of(15).in(Meters);
        /**
         * Distance from the floor to the front limelight lens in meters.
         */
        public static final double FRONT_LIMELIGHT_UP_OFFSET = Inches.of(8.5).in(Meters);

        public static final String BACK_LIMELIGHT_NAME = "limelight-back";
        /**
         * Distance from the center of the robot to the front limelight lens in meters.
         */
        public static final double BACK_LIMELIGHT_FORWARD_OFFSET =
                Inches.of(-7.25).in(Meters);
        /**
         * Distance from the floor to the front limelight lens in meters.
         */
        public static final double BACK_LIMELIGHT_UP_OFFSET = Inches.of(20).in(Meters);
        /**
         * Units: degrees
         */
        public static final double BACK_LIMELIGHT_YAW = 180;

        /*
         * Used for setting the limelight's fiducial 3D offset.
         * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#point-of-interest-tracking
         *
         * These measurements were found by importing the Field CAD to Onshape, generating a top-down drawing,
         * and using the dimension tool from the edge (for FORWARD) or centerline (for RIGHT) of the reef wall to the centermark of the L4 branch.
         */
        public static final double TAG_TO_LEFT_TREE_FORWARD_OFFSET =
                Inches.of(-2.052).in(Meters);
        public static final double TAG_TO_LEFT_TREE_RIGHT_OFFSET =
                Inches.of(-6.470).in(Meters);
        public static final double TAG_TO_RIGHT_TREE_FORWARD_OFFSET =
                Inches.of(-2.007).in(Meters);
        public static final double TAG_TO_RIGHT_TREE_RIGHT_OFFSET =
                Inches.of(6.468).in(Meters);

        public static final double[] FRONT_TAG_DISTANCE_TABLE = new double[] {0.495, 1.028, 1.503, 2.2};
        public static final double[] FRONT_X_STDDEV_TABLE = new double[] {0.00006, 0.00045, 0.00075, 0.0028};
        public static final double[] FRONT_Y_STDDEV_TABLE = new double[] {0.00015, 0.0002, 0.0003, 0.001};
        public static final double[] BACK_TAG_DISTANCE_TABLE = new double[] {};
        public static final double[] BACK_X_STDDEV_TABLE = new double[] {};
        public static final double[] BACK_Y_STDDEV_TABLE = new double[] {};
    }

    /**
     * This class contains information about the field, like the positions of game elements and Apriltags.
     * <p>
     * This information comes from <a href="https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf">the FRC Field Drawings.</a>
     * <p>
     * The x and y coordinates were found by importing the Field CAD to Onshape, generating a top-down drawing,
     * and using the dimension tool.
     */
    public static class FieldConstants {
        private FieldConstants() {}

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(Inches.of(176.183), Inches.of(158.500));
        public static final Translation2d RED_REEF_CENTER = new Translation2d(Inches.of(513.568), Inches.of(158.500));

        private static final Rotation2d k60deg = Rotation2d.fromDegrees(60);
        private static final Rotation2d k120deg = Rotation2d.fromDegrees(120);
        private static final Rotation2d k240deg = Rotation2d.fromDegrees(240);
        private static final Rotation2d k300deg = Rotation2d.fromDegrees(300);

        /**
         * See page 24 of <a href="https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">the game manual</a> to understand what each letter means.
         * The rotation represents which way the robot needs to face to be aligned with the tree's wall.
         */
        public static final List<Pose2d> BLUE_REEF_TREE_POSITIONS = List.of(
                // A
                new Pose2d(Inches.of(145.490), Inches.of(164.968), Rotation2d.kZero),
                // B
                new Pose2d(Inches.of(145.445), Inches.of(152.030), Rotation2d.kZero),
                // C
                new Pose2d(Inches.of(155.234), Inches.of(135.152), k60deg),
                // D
                new Pose2d(Inches.of(166.416), Inches.of(128.645), k60deg),
                // E
                new Pose2d(Inches.of(185.927), Inches.of(128.684), k120deg),
                // F
                new Pose2d(Inches.of(197.154), Inches.of(135.113), k120deg),
                // G
                new Pose2d(Inches.of(206.876), Inches.of(152.030), Rotation2d.k180deg),
                // H
                new Pose2d(Inches.of(206.921), Inches.of(164.968), Rotation2d.k180deg),
                // I
                new Pose2d(Inches.of(197.132), Inches.of(181.846), k240deg),
                // J
                new Pose2d(Inches.of(185.950), Inches.of(188.354), k240deg),
                // K
                new Pose2d(Inches.of(166.439), Inches.of(188.315), k300deg),
                // L
                new Pose2d(Inches.of(155.212), Inches.of(181.885), k300deg));

        /**
         * See page 24 of <a href="https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">the game manual</a> to understand what each letter means.
         * The rotation represents which way the robot needs to face to be aligned with the tree's wall.
         */
        public static final List<Pose2d> RED_REEF_TREE_POSITIONS = List.of(
                // A
                new Pose2d(Inches.of(544.261), Inches.of(152.030), Rotation2d.k180deg),
                // B
                new Pose2d(Inches.of(544.306), Inches.of(164.968), Rotation2d.k180deg),
                // C
                new Pose2d(Inches.of(534.517), Inches.of(181.846), k240deg),
                // D
                new Pose2d(Inches.of(523.335), Inches.of(188.354), k240deg),
                // E
                new Pose2d(Inches.of(503.823), Inches.of(188.315), k300deg),
                // F
                new Pose2d(Inches.of(492.597), Inches.of(181.885), k300deg),
                // G
                new Pose2d(Inches.of(482.875), Inches.of(164.968), Rotation2d.kZero),
                // H
                new Pose2d(Inches.of(482.830), Inches.of(152.030), Rotation2d.kZero),
                // I
                new Pose2d(Inches.of(492.619), Inches.of(135.152), k60deg),
                // J
                new Pose2d(Inches.of(503.801), Inches.of(128.645), k60deg),
                // K
                new Pose2d(Inches.of(523.312), Inches.of(128.684), k120deg),
                // L
                new Pose2d(Inches.of(534.539), Inches.of(135.113), k120deg));

        /** Distance from tree to reef wall */
        private static final Distance TREE_TO_REEF_WALL_DISTANCE = Inches.of(2);

        /** Distance from tree to center of robot in meters */
        private static final double TREE_TO_ROBOT_DISTANCE = TREE_TO_REEF_WALL_DISTANCE
                .plus(DriveConstants.CENTER_TO_BUMPER_LENGTH)
                .in(Meters);

        /**
         * This is a list of positions for the robot to drive to so that it is right in front of the tree, and facing the tree's wall.
         */
        public static final List<Pose2d> BLUE_REEF_TREE_AIMING_POSITIONS = List.of(
                // A
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(0), new Translation2d(TREE_TO_ROBOT_DISTANCE, Rotation2d.k180deg)),
                // B
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(1), new Translation2d(TREE_TO_ROBOT_DISTANCE, Rotation2d.k180deg)),
                // C
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(2), new Translation2d(TREE_TO_ROBOT_DISTANCE, k240deg)),
                // D
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(3), new Translation2d(TREE_TO_ROBOT_DISTANCE, k240deg)),
                // E
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(4), new Translation2d(TREE_TO_ROBOT_DISTANCE, k300deg)),
                // F
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(5), new Translation2d(TREE_TO_ROBOT_DISTANCE, k300deg)),
                // G
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(6), new Translation2d(TREE_TO_ROBOT_DISTANCE, Rotation2d.kZero)),
                // H
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(7), new Translation2d(TREE_TO_ROBOT_DISTANCE, Rotation2d.kZero)),
                // I
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(8), new Translation2d(TREE_TO_ROBOT_DISTANCE, k60deg)),
                // J
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(9), new Translation2d(TREE_TO_ROBOT_DISTANCE, k60deg)),
                // K
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(10), new Translation2d(TREE_TO_ROBOT_DISTANCE, k120deg)),
                // L
                Aiming.translatePose(
                        BLUE_REEF_TREE_POSITIONS.get(11), new Translation2d(TREE_TO_ROBOT_DISTANCE, k120deg)));

        /**
         * This is a list of positions for the robot to drive to so that it is right in front of the tree, and facing the tree's wall.
         */
        public static final List<Pose2d> RED_REEF_TREE_AIMING_POSITIONS = List.of(
                // A
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(0), new Translation2d(TREE_TO_ROBOT_DISTANCE, Rotation2d.kZero)),
                // B
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(1), new Translation2d(TREE_TO_ROBOT_DISTANCE, Rotation2d.kZero)),
                // C
                Aiming.translatePose(RED_REEF_TREE_POSITIONS.get(2), new Translation2d(TREE_TO_ROBOT_DISTANCE, k60deg)),
                // D
                Aiming.translatePose(RED_REEF_TREE_POSITIONS.get(3), new Translation2d(TREE_TO_ROBOT_DISTANCE, k60deg)),
                // E
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(4), new Translation2d(TREE_TO_ROBOT_DISTANCE, k120deg)),
                // F
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(5), new Translation2d(TREE_TO_ROBOT_DISTANCE, k120deg)),
                // G
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(6), new Translation2d(TREE_TO_ROBOT_DISTANCE, Rotation2d.k180deg)),
                // H
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(7), new Translation2d(TREE_TO_ROBOT_DISTANCE, Rotation2d.k180deg)),
                // I
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(8), new Translation2d(TREE_TO_ROBOT_DISTANCE, k240deg)),
                // J
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(9), new Translation2d(TREE_TO_ROBOT_DISTANCE, k240deg)),
                // K
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(10), new Translation2d(TREE_TO_ROBOT_DISTANCE, k300deg)),
                // L
                Aiming.translatePose(
                        RED_REEF_TREE_POSITIONS.get(11), new Translation2d(TREE_TO_ROBOT_DISTANCE, k300deg)));

        /**
         * Positions of the Apriltags for logging currently visible vision targets in AdvantageScope.
         * <p>
         * IMPORTANT: Index 0 corresponds to tag id 1. Index 21 corresponds to tag id 22.
         * Basically, index into the array by subtracting one from the id.
         */
        public static final Translation3d[] APRILTAG_POSES = {
            new Translation3d(Inches.of(657.37), Inches.of(25.80), Inches.of(58.50)),
            new Translation3d(Inches.of(657.37), Inches.of(291.20), Inches.of(58.50)),
            new Translation3d(Inches.of(455.15), Inches.of(317.15), Inches.of(51.25)),
            new Translation3d(Inches.of(365.20), Inches.of(241.64), Inches.of(573.54)),
            new Translation3d(Inches.of(365.20), Inches.of(75.39), Inches.of(73.54)),
            new Translation3d(Inches.of(530.49), Inches.of(130.17), Inches.of(12.13)),
            new Translation3d(Inches.of(546.87), Inches.of(158.50), Inches.of(12.13)),
            new Translation3d(Inches.of(530.49), Inches.of(186.83), Inches.of(12.13)),
            new Translation3d(Inches.of(497.77), Inches.of(186.83), Inches.of(12.13)),
            new Translation3d(Inches.of(481.39), Inches.of(158.50), Inches.of(12.13)),
            new Translation3d(Inches.of(497.77), Inches.of(130.17), Inches.of(12.13)),
            new Translation3d(Inches.of(33.51), Inches.of(25.80), Inches.of(58.50)),
            new Translation3d(Inches.of(33.51), Inches.of(291.20), Inches.of(58.50)),
            new Translation3d(Inches.of(325.68), Inches.of(241.64), Inches.of(73.54)),
            new Translation3d(Inches.of(325.68), Inches.of(75.39), Inches.of(73.54)),
            new Translation3d(Inches.of(235.73), Inches.of(-0.15), Inches.of(51.25)),
            new Translation3d(Inches.of(160.39), Inches.of(130.17), Inches.of(12.13)),
            new Translation3d(Inches.of(144.00), Inches.of(158.50), Inches.of(12.13)),
            new Translation3d(Inches.of(160.39), Inches.of(186.83), Inches.of(12.13)),
            new Translation3d(Inches.of(193.10), Inches.of(186.83), Inches.of(12.13)),
            new Translation3d(Inches.of(209.49), Inches.of(158.50), Inches.of(12.13)),
            new Translation3d(Inches.of(193.10), Inches.of(130.17), Inches.of(12.13))
        };

        /** Log this array to AdvantageScope when there are no tags reported by the limelight. */
        public static final Translation3d[] NO_VISIBLE_TAGS = new Translation3d[0];

        public static final double[] NO_TAG_DISTANCES = new double[0];

        /**
         * Rotations of the reef walls for for aligning perpendicular to them. The rotations are in no particular order.
         * <p>
         * These rotations correspond to the Z-Rotation of the reef wall tags on <a href="https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf">FRC Field Drawings</a>
         * plus 180 degrees.
         */
        public static final List<Rotation2d> REEF_WALL_ROTATIONS =
                List.of(Rotation2d.kZero, k60deg, k120deg, Rotation2d.k180deg, k240deg, k300deg);

        /**
         * Rotations of the Apriltags for aligning perpendicular to them.
         * <p>
         * These rotations correspond to the Z-Rotation on <a href="https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf">FRC Field Drawings</a>
         * plus 180 degrees.
         * <p>
         * Index into the array with the id number starting from 1.
         */
        public static final Rotation2d[] APRILTAG_ROTATIONS = {
            Rotation2d.kZero,
            new Rotation2d(Degrees.of(126).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(234).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(270).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(0).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(0).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(300).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(0).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(60).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(120).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(180).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(240).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(54).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(306).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(180).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(180).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(90).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(240).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(180).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(120).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(60).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(0).plus(Degrees.of(180))),
            new Rotation2d(Degrees.of(300).plus(Degrees.of(180))),
        };

        public static final int MINIMUM_RED_REEF_TAG_ID = 6;
        public static final int MAXIMUM_RED_REEF_TAG_ID = 11;
        public static final int MINIMUM_BLUE_REEF_TAG_ID = 17;
        public static final int MAXIMUM_BLUE_REEF_TAG_ID = 22;

        /** Distance from wall to the middle of the barge. */
        public static final Distance BARGE_CENTER_X_DISTANCE = Inches.of(344.875);
        /** Distance from the middle of the barge to the tape under the barge. */
        public static final Translation2d BARGE_TAPE_DISTANCE = new Translation2d(Inches.of(21), Meters.zero());
        /** Distance from the colored part of the barge to the center of the cage. */
        private static final Distance SEMICIRCLE_TO_CAGE_DISTANCE = Inches.of(1.250);

        /**
         * A list of red cage positions.
         * These were found by generating a drawing in Onshape.
         * <p>
         * For x, I took the distance from the wall to the center of the barge.
         * <p>
         * For y, I took the distance from the wall to each of the barge's red semicircles minus the distance from the semicircles to the center of the cage.
         */
        public static final List<Translation2d> RED_CAGE_POSITIONS = List.of(
                new Translation2d(BARGE_CENTER_X_DISTANCE, Inches.of(32.432).minus(SEMICIRCLE_TO_CAGE_DISTANCE)),
                new Translation2d(BARGE_CENTER_X_DISTANCE, Inches.of(75.375).minus(SEMICIRCLE_TO_CAGE_DISTANCE)),
                new Translation2d(BARGE_CENTER_X_DISTANCE, Inches.of(118.312).minus(SEMICIRCLE_TO_CAGE_DISTANCE)));

        /**
         * A list of blue cage positions.
         * These were found by generating a drawing in Onshape.
         * <p>
         * For x, I took the distance from the wall to the center of the barge.
         * <p>
         * For y, I took the distance from the wall to each of the barge's blue semicircles plus the distance from the semicircles to the center of the cage.
         * <p>
         * The rotation represents the angle the robot needs to face so that the robot's side opening aligns with the cage.
         */
        public static final List<Translation2d> BLUE_CAGE_POSITIONS = List.of(
                new Translation2d(BARGE_CENTER_X_DISTANCE, Inches.of(198.688).plus(SEMICIRCLE_TO_CAGE_DISTANCE)),
                new Translation2d(BARGE_CENTER_X_DISTANCE, Inches.of(241.625).plus(SEMICIRCLE_TO_CAGE_DISTANCE)),
                new Translation2d(BARGE_CENTER_X_DISTANCE, Inches.of(284.568).plus(SEMICIRCLE_TO_CAGE_DISTANCE)));
    }

    /** A map of CAN ids to motor names for <a href="https://docs.advantagescope.org/more-features/urcl">URCL</a>. */
    public static final Map<Integer, String> REV_CAN_ID_ALIASES = Map.of(
            ClimbConstants.BACK_ID,
            "Climb-Back",
            ClimbConstants.FRONT_ID,
            "Climb-Front",
            CatcherConstants.ARM_ID,
            "Catcher-Arm",
            CatcherConstants.WHEELS_ID,
            "Catcher-Wheels",
            IntakeConstants.ARM_ID,
            "Intake-Arm",
            IntakeConstants.WHEEL_LEFT_ID,
            "Intake-Wheel-Left",
            IntakeConstants.WHEEL_RIGHT_ID,
            "Intake-Wheel-Right");
}
