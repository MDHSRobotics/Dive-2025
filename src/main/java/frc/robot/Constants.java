// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.IntakeConstants;
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

    /**
     * The update period of the robot in seconds.
     */
    public static final double UPDATE_PERIOD = 0.02;

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
        public static final double FRONT_LIMELIGHT_UP_OFFSET = Inches.of(10.125).in(Meters);

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

        public static final double FRONT_X_STD_DEV = 0.04;
        public static final double FRONT_Y_STD_DEV = 0.0017;
        public static final double BACK_X_STD_DEV = 0.7;
        public static final double BACK_Y_STD_DEV = 0.7;
        public static Vector<N3> FRONT_STD_DEVS = VecBuilder.fill(FRONT_X_STD_DEV, FRONT_Y_STD_DEV, Double.MAX_VALUE);
        public static Vector<N3> BACK_STD_DEVS = VecBuilder.fill(BACK_X_STD_DEV, BACK_Y_STD_DEV, Double.MAX_VALUE);
        // public static final double[] FRONT_TAG_DISTANCE_TABLE = new double[] {0.495, 1.028, 1.503, 2.2, 5.2};
        // public static final double[] FRONT_X_STDDEV_TABLE = new double[] {0.00006, 0.00045, 0.00075, 0.0028, 0.04};
        // public static final double[] FRONT_Y_STDDEV_TABLE = new double[] {0.00015, 0.0002, 0.0003, 0.001, 0.0017};
        // public static final double[] BACK_TAG_DISTANCE_TABLE = new double[] {6.0};
        // public static final double[] BACK_X_STDDEV_TABLE = new double[] {0.07};
        // public static final double[] BACK_Y_STDDEV_TABLE = new double[] {0.0025};
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

        public static final AprilTagFieldLayout APRILTAGS =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        /**
         * Rotations of the Apriltags for aligning perpendicular to them.
         * <p>
         * These rotations correspond to the Z-Rotation of the tags plus 180 degrees.
         * <p>
         * Index into the array with the id number starting from 1.
         */
        public static final Rotation2d[] APRILTAG_ROTATIONS = {
            Rotation2d.kZero,
            APRILTAGS.getTagPose(1).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(2).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(3).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(4).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(5).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(6).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(7).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(8).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(9).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(10).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(11).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(12).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(13).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(14).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(15).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(16).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(17).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(18).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(19).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(20).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(21).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
            APRILTAGS.getTagPose(22).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
        };

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(
                (APRILTAGS.getTagPose(21).orElseThrow().getX()
                                + APRILTAGS.getTagPose(18).orElseThrow().getX())
                        / 2.0,
                APRILTAGS.getTagPose(21).orElseThrow().getY());
        public static final Translation2d RED_REEF_CENTER = new Translation2d(
                (APRILTAGS.getTagPose(7).orElseThrow().getX()
                                + APRILTAGS.getTagPose(10).orElseThrow().getX())
                        / 2.0,
                APRILTAGS.getTagPose(7).orElseThrow().getY());

        /**
         * The distance from a reef apriltag to either of its two trees in meters.
         * This is only the distance along the face of the reef, and is not two-dimensional.
         */
        private static final double REEF_TAG_TO_TREE_DISTANCE = Inches.of(6.47).in(Meters);
        /** Distance from tree to center of robot in meters */
        private static final double REEF_WALL_TO_ROBOT_DISTANCE = DriveConstants.CENTER_TO_BUMPER_LENGTH.in(Meters);

        private static final Rotation2d k60deg = Rotation2d.fromDegrees(60.0);
        private static final Rotation2d k120deg = Rotation2d.fromDegrees(120.0);
        private static final Rotation2d k240deg = Rotation2d.fromDegrees(240.0);
        private static final Rotation2d k300deg = Rotation2d.fromDegrees(300.0);

        /**
         * An approximation of the tree positions.
         * These translations actually represent the points of the reef wall that the trees sit behind.
         */
        private static final Translation2d[] BLUE_REEF_TREE_POSITIONS = new Translation2d[] {
            // A
            APRILTAGS
                    .getTagPose(18)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[18].plus(Rotation2d.kCCW_90deg))),
            // B
            APRILTAGS
                    .getTagPose(18)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[18].plus(Rotation2d.kCW_90deg))),
            // C
            APRILTAGS
                    .getTagPose(17)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[17].plus(Rotation2d.kCCW_90deg))),
            // D
            APRILTAGS
                    .getTagPose(17)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[17].plus(Rotation2d.kCW_90deg))),
            // E
            APRILTAGS
                    .getTagPose(22)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[22].plus(Rotation2d.kCCW_90deg))),
            // F
            APRILTAGS
                    .getTagPose(22)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[22].plus(Rotation2d.kCW_90deg))),
            // G
            APRILTAGS
                    .getTagPose(21)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[21].plus(Rotation2d.kCCW_90deg))),
            // H
            APRILTAGS
                    .getTagPose(21)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[21].plus(Rotation2d.kCW_90deg))),
            // I
            APRILTAGS
                    .getTagPose(20)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[20].plus(Rotation2d.kCCW_90deg))),
            // J
            APRILTAGS
                    .getTagPose(20)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[20].plus(Rotation2d.kCW_90deg))),
            // K
            APRILTAGS
                    .getTagPose(19)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[19].plus(Rotation2d.kCCW_90deg))),
            // L
            APRILTAGS
                    .getTagPose(19)
                    .orElseThrow()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(
                            REEF_TAG_TO_TREE_DISTANCE, APRILTAG_ROTATIONS[19].plus(Rotation2d.kCW_90deg))),
        };

        /**
         * This is a list of positions for the robot to drive to so that it is right in front of the tree, and facing the tree's wall.
         */
        public static final List<Pose2d> BLUE_REEF_TREE_AIMING_POSITIONS = List.of(
                // A
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[0].plus(
                                new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, Rotation2d.k180deg)),
                        Rotation2d.kZero),
                // B
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[1].plus(
                                new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, Rotation2d.k180deg)),
                        Rotation2d.kZero),
                // C
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[2].plus(new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, k240deg)),
                        k60deg),
                // D
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[3].plus(new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, k240deg)),
                        k60deg),
                // E
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[4].plus(new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, k300deg)),
                        k120deg),
                // F
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[5].plus(new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, k300deg)),
                        k120deg),
                // G
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[6].plus(
                                new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, Rotation2d.kZero)),
                        Rotation2d.k180deg),
                // H
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[7].plus(
                                new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, Rotation2d.kZero)),
                        Rotation2d.k180deg),
                // I
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[8].plus(new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, k60deg)),
                        k240deg),
                // J
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[9].plus(new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, k60deg)),
                        k240deg),
                // K
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[10].plus(new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, k120deg)),
                        k300deg),
                // L
                new Pose2d(
                        BLUE_REEF_TREE_POSITIONS[11].plus(new Translation2d(REEF_WALL_TO_ROBOT_DISTANCE, k120deg)),
                        k300deg));

        /**
         * This is a list of positions for the robot to drive to so that it is right in front of the tree, and facing the tree's wall.
         */
        public static final List<Pose2d> RED_REEF_TREE_AIMING_POSITIONS = List.of(
                // A
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(0)),
                // B
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(1)),
                // C
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(2)),
                // D
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(3)),
                // E
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(4)),
                // F
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(5)),
                // G
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(6)),
                // H
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(7)),
                // I
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(8)),
                // J
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(9)),
                // K
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(10)),
                // L
                FlippingUtil.flipFieldPose(BLUE_REEF_TREE_AIMING_POSITIONS.get(11)));

        /** Log this array to AdvantageScope when there are no tags reported by the limelight. */
        public static final Translation3d[] NO_VISIBLE_TAGS = new Translation3d[0];
        /** Log this array to AdvantageScope when there are no tags reported by the limelight. */
        public static final double[] NO_TAG_DISTANCES = new double[0];

        /**
         * Rotations of the reef walls for for aligning perpendicular to them. The rotations are in no particular order.
         * <p>
         * These rotations correspond to the Z-Rotation of the reef wall tags on <a href="https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf">FRC Field Drawings</a>
         * plus 180 degrees.
         */
        public static final List<Rotation2d> REEF_WALL_ROTATIONS =
                List.of(Rotation2d.kZero, k60deg, k120deg, Rotation2d.k180deg, k240deg, k300deg);

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

        private static final double CORAL_STATION_TO_ROBOT_DISTANCE =
                DriveConstants.CENTER_TO_BUMPER_LENGTH.plus(Inches.of(6)).in(Meters);

        public static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(
                new Pose2d(
                        APRILTAGS
                                .getTagPose(13)
                                .orElseThrow()
                                .getTranslation()
                                .toTranslation2d()
                                .plus(new Translation2d(
                                        CORAL_STATION_TO_ROBOT_DISTANCE,
                                        APRILTAG_ROTATIONS[13].plus(Rotation2d.k180deg))),
                        APRILTAG_ROTATIONS[13]),
                new Pose2d(
                        APRILTAGS
                                .getTagPose(12)
                                .orElseThrow()
                                .getTranslation()
                                .toTranslation2d()
                                .plus(new Translation2d(
                                        CORAL_STATION_TO_ROBOT_DISTANCE,
                                        APRILTAG_ROTATIONS[12].plus(Rotation2d.k180deg))),
                        APRILTAG_ROTATIONS[12]));

        public static final List<Pose2d> RED_CORAL_STATION_POSES = List.of(
                new Pose2d(
                        APRILTAGS
                                .getTagPose(1)
                                .orElseThrow()
                                .getTranslation()
                                .toTranslation2d()
                                .plus(new Translation2d(
                                        CORAL_STATION_TO_ROBOT_DISTANCE,
                                        APRILTAG_ROTATIONS[1].plus(Rotation2d.k180deg))),
                        APRILTAG_ROTATIONS[1]),
                new Pose2d(
                        APRILTAGS
                                .getTagPose(2)
                                .orElseThrow()
                                .getTranslation()
                                .toTranslation2d()
                                .plus(new Translation2d(
                                        CORAL_STATION_TO_ROBOT_DISTANCE,
                                        APRILTAG_ROTATIONS[2].plus(Rotation2d.k180deg))),
                        APRILTAG_ROTATIONS[2]));
    }

    /** A map of CAN ids to motor names for <a href="https://docs.advantagescope.org/more-features/urcl">URCL</a>. */
    public static final Map<Integer, String> REV_CAN_ID_ALIASES = Map.of(
            ClimbConstants.BACK_ID,
            "Climb-Back",
            ClimbConstants.FRONT_ID,
            "Climb-Front",
            ElevatorConstants.ARM_ID,
            "Catcher-Arm",
            ElevatorConstants.WHEELS_ID,
            "Catcher-Wheels",
            IntakeConstants.ARM_ID,
            "Intake-Arm",
            IntakeConstants.WHEEL_LEFT_ID,
            "Intake-Wheel-Left",
            IntakeConstants.WHEEL_RIGHT_ID,
            "Intake-Wheel-Right");
}
