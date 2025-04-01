package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

public class ClimbConstants {
    private ClimbConstants() {}

    /** The CAN id of the left motor. */
    public static final int BACK_ID = 2;
    /** The CAN id of the right motor. */
    public static final int FRONT_ID = 3;

    public static final int CAGE_BEAM_BREAK_DIO_CHANNEL = 7;

    /**
     * The current limit for the climb motors in amps.
     * This is currently set to the value suggested by
     * <a href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV for the NEO Vortex.</a>
     */
    public static final int CURRENT_LIMIT = 80;

    /** The furthest safe angle that is away from the cage in radians. */
    public static final double BACK_MAX_LIMIT = 3.463;
    /** The angle that pushes the cage down as far as possible in radians. */
    public static final double BACK_MIN_LIMIT = 0.533;

    /** The furthest safe angle that is away from the cage in radians. */
    public static final double FRONT_MAX_LIMIT = 4.561;
    /** The angle that pushes the cage down as far as possible in radians. */
    public static final double FRONT_MIN_LIMIT = 1.496;

    /**
     * The position of the absolute encoder (before any position conversion factor) that reports 0.
     * This is currently set outside the motor's range of motion to prevent the position wrapping around from 1 to 0.
     */
    public static final double BACK_ZERO_OFFSET = 0.4;

    /**
     * The position of the absolute encoder (before any position conversion factor) that reports 0.
     * This is currently set outside the motor's range of motion to prevent the position wrapping around from 1 to 0.
     */
    public static final double FRONT_ZERO_OFFSET = 0;

    /**
     * The conversion of motor input rotations to climb hook output radians.
     */
    public static final double POSITION_CONVERSION_FACTOR = Rotations.one().in(Radians);
    /**
     * The conversion of motor input rotations per minute to climb hook output radians per second.
     */
    public static final double VELOCITY_CONVERSION_FACTOR =
            Rotations.per(Minute).one().in(RadiansPerSecond);
}
