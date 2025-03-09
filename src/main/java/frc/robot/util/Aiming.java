package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Aiming {
    private Aiming() {}

    /**
     * Returns the nearest Rotation2d to the given rotation from a list of rotations.
     * <p>
     * Based off of the rotational part of {@link edu.wpi.first.math.geometry.Pose2d#nearest(List) WPILib's nearest pose method.}
     * @param rotation The rotation being used to find the nearest one from the list.
     * @param otherRotations The list of rotations to find the nearest.
     * @return The nearest Rotation2d from the list.
     */
    public static Rotation2d nearestRotation(Rotation2d rotation, List<Rotation2d> otherRotations) {
        return Collections.min(
                otherRotations,
                Comparator.comparing(
                        (Rotation2d other) -> Math.abs(rotation.minus(other).getRadians())));
    }

    /**
     * Translates a pose based on a given translation and returns a new pose.
     * @param pose The pose to translate
     * @param translation The amount to move the pose by
     */
    public static Pose2d translatePose(Pose2d pose, Translation2d translation) {
        return new Pose2d(pose.getX() + translation.getX(), pose.getY() + translation.getY(), pose.getRotation());
    }

    /**
     * Takes a given apriltag ID, and returns whether or not it is a reef tag.
     */
    public static boolean isReefTag(int id) {
        return (id >= FieldConstants.MINIMUM_RED_REEF_TAG_ID && id <= FieldConstants.MAXIMUM_RED_REEF_TAG_ID)
                || (id >= FieldConstants.MINIMUM_BLUE_REEF_TAG_ID && id <= FieldConstants.MAXIMUM_BLUE_REEF_TAG_ID);
    }

    public static boolean isNearCoralStation(Translation2d robotPosition) {
        Alliance alliance = DriverStation.getAlliance().orElseThrow();
        if (alliance == Alliance.Blue) {
            Translation2d coralStationLeft = FieldConstants.APRILTAG_POSES[13].toTranslation2d();
            Translation2d coralStationRight = FieldConstants.APRILTAG_POSES[12].toTranslation2d();
            double distanceLeft = coralStationLeft.getDistance(robotPosition);
            // System.out.println("Distance left: " + distanceLeft);
            double distanceRight = coralStationRight.getDistance(robotPosition);
            // System.out.println("Distance right: " + distanceRight);
            return distanceLeft < 1.0 || distanceRight < 1.0;
        } else {
            Translation2d coralStationLeft = FieldConstants.APRILTAG_POSES[1].toTranslation2d();
            Translation2d coralStationRight = FieldConstants.APRILTAG_POSES[1].toTranslation2d();
            double distanceLeft = coralStationLeft.getDistance(robotPosition);
            // System.out.println("Distance left: " + distanceLeft);
            double distanceRight = coralStationRight.getDistance(robotPosition);
            // System.out.println("Distance right: " + distanceRight);
            return distanceLeft < 1.0 || distanceRight < 1.0;
        }
    }
}
