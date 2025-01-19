package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
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
     * Takes a given apriltag ID, and returns whether or not it is a reef tag.
     */
    public static boolean isReefTag(int id) {
        return (id >= FieldConstants.MINIMUM_RED_REEF_TAG_ID && id <= FieldConstants.MAXIMUM_RED_REEF_TAG_ID)
                || (id >= FieldConstants.MINIMUM_BLUE_REEF_TAG_ID && id <= FieldConstants.MAXIMUM_BLUE_REEF_TAG_ID);
    }
}
