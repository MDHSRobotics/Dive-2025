// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.drive.TunerConstants;
import java.util.List;

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

    /**
     * Contains constants for the swerve drive that haven't been specified in {@link frc.robot.subsystems.drive.TunerConstants TunerConstants}.
     * <p>
     * Max linear speed is restated here because we want to avoid constant unit conversions in the main robot loop.
     */
    public static class DriveConstants {
        private DriveConstants() {}
        /*
         * Front Right to Front Left: 1ft 9in
         * Front Left to Back Left: 2ft
         */

        /** Max linear speed of the robot in meters per second. This still needs to be tuned. */
        public static final double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        /** Max angular rate of the robot in radians per second. This still needs to be tuned. */
        public static final double MAX_ANGULAR_RATE =
                RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /**
         * Constraints for the motion profiles used in custom swerve requests.
         * This still needs to be tuned.
         */
        public static final TrapezoidProfile.Constraints ANGULAR_MOTION_CONSTRAINTS =
                new TrapezoidProfile.Constraints(MAX_ANGULAR_RATE, 0);

        /**
         * Proportional gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">heading PID controller</a>
         * used in custom swerve requests.
         * This still needs to be tuned.
         */
        public static final double K_P = 0;

        /**
         * Derivative gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">heading PID controller</a>
         * used in custom swerve requests.
         * This still needs to be tuned.
         */
        public static final double K_D = 0;

        /**
         * Goal tolerance for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">heading PID controller</a>
         * used in custom swerve requests.
         * This still needs to be tuned.
         */
        public static final Angle GOAL_TOLERANCE = Degrees.of(10);
    }

    public static class VisionConstants {
        private VisionConstants() {}

        public static final String LIMELIGHT_NAME = "limelight-front";

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

        /**
         * See page 24 of <a href="https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">the game manual</a> to understand what each letter means.
         */
        public static final List<Translation2d> BLUE_REEF_TREE_POSITIONS = List.of(
                // A
                new Translation2d(Inches.of(145.490), Inches.of(164.968)),
                // B
                new Translation2d(Inches.of(145.445), Inches.of(152.030)),
                // C
                new Translation2d(Inches.of(155.234), Inches.of(135.152)),
                // D
                new Translation2d(Inches.of(166.416), Inches.of(128.645)),
                // E
                new Translation2d(Inches.of(185.927), Inches.of(128.684)),
                // F
                new Translation2d(Inches.of(197.154), Inches.of(135.113)),
                // G
                new Translation2d(Inches.of(206.876), Inches.of(152.030)),
                // H
                new Translation2d(Inches.of(206.921), Inches.of(164.968)),
                // I
                new Translation2d(Inches.of(197.132), Inches.of(181.846)),
                // J
                new Translation2d(Inches.of(185.950), Inches.of(188.354)),
                // K
                new Translation2d(Inches.of(166.439), Inches.of(188.315)),
                // L
                new Translation2d(Inches.of(155.212), Inches.of(181.885)));

        /**
         * See page 24 of <a href="https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">the game manual</a> to understand what each letter means.
         */
        public static final List<Translation2d> RED_REEF_TREE_POSITIONS = List.of(
                // A
                new Translation2d(Inches.of(544.261), Inches.of(152.030)),
                // B
                new Translation2d(Inches.of(544.306), Inches.of(164.968)),
                // C
                new Translation2d(Inches.of(534.517), Inches.of(181.846)),
                // D
                new Translation2d(Inches.of(523.335), Inches.of(188.354)),
                // E
                new Translation2d(Inches.of(503.823), Inches.of(188.315)),
                // F
                new Translation2d(Inches.of(492.597), Inches.of(181.885)),
                // G
                new Translation2d(Inches.of(482.875), Inches.of(164.968)),
                // H
                new Translation2d(Inches.of(482.830), Inches.of(152.030)),
                // I
                new Translation2d(Inches.of(492.619), Inches.of(135.152)),
                // J
                new Translation2d(Inches.of(503.801), Inches.of(128.645)),
                // K
                new Translation2d(Inches.of(523.312), Inches.of(128.684)),
                // L
                new Translation2d(Inches.of(534.539), Inches.of(135.113)));

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

        /**
         * Rotations of the reef walls for for aligning perpendicular to them. The rotations are in no particular order.
         * <p>
         * These rotations correspond to the Z-Rotation of the reef wall tags on <a href="https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf">FRC Field Drawings</a> plus 180 degrees.
         */
        public static final List<Rotation2d> REEF_WALL_ROTATIONS = List.of(
                new Rotation2d(Degrees.of(180).plus(Degrees.of(180))),
                new Rotation2d(Degrees.of(240).plus(Degrees.of(180))),
                new Rotation2d(Degrees.of(300).plus(Degrees.of(180))),
                new Rotation2d(Degrees.of(0).plus(Degrees.of(180))),
                new Rotation2d(Degrees.of(60).plus(Degrees.of(180))),
                new Rotation2d(Degrees.of(120).plus(Degrees.of(180))));

        /**
         * Rotations of the Apriltags for aligning perpendicular to them.
         * <p>
         * These rotations correspond to the Z-Rotation on <a href="https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf">FRC Field Drawings</a> plus 180 degrees.
         * <p>
         * IMPORTANT: Index 0 corresponds to tag id 1. Index 21 corresponds to tag id 22.
         * Basically, index into the array by subtracting one from the id.
         */
        public static final Rotation2d[] APRILTAG_ROTATIONS = {
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
    }
}
