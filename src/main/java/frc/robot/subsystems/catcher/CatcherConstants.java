package frc.robot.subsystems.catcher;

import static edu.wpi.first.units.Units.*;

public class CatcherConstants {
    private CatcherConstants() {}

    public static final int ARM_ID = 4;
    public static final int WHEELS_ID = 5;

    /**
     * The current limit for the arm and wheels in amps.
     * This is currently set to the value suggested by
     * <a href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV for the NEO Vortex.</a>
     */
    public static final int CURRENT_LIMIT = (int) Amps.of(80).in(Amps);

    public static final double ARM_MIN_LIMIT = Radians.of(2.22).in(Radians);
    public static final double ARM_MAX_LIMIT = Radians.of(5.80).in(Radians);

    /**
     * The conversion of motor input rotations to arm output radians.
     */
    public static final double ARM_POSITION_CONVERSION_FACTOR = Rotations.of(1).in(Radians);
    /**
     * The conversion of motor input rotations per minute to arm output radians per second.
     */
    public static final double ARM_VELOCITY_CONVERSION_FACTOR =
            Rotations.per(Minute).of(1).in(RadiansPerSecond);

    /**
     * The position of the absolute encoder (before any position conversion factor) that reports 0.
     * This is currently set outside the arm's range of motion to prevent the position wrapping around from 1 to 0.
     */
    public static final double ARM_ZERO_OFFSET = 0.75;

    /**
     * Proportional gain for the <a href="https://docs.revrobotics.com/revlib/spark/closed-loop#closed-loop-control-with-spark-motor-controllers">internal closed loop controller</a>
     * in duty cycle (percent out) per radian.
     * This still needs to be tuned.
     */
    public static final double K_P = 0.225;

    // Common catcher positions
    /**
     * The position that aligns the arm with the trough in radians.
     */
    public static final double TROUGH_POSITION = Radians.of(5.32).in(Radians);

    public static final double L1_POSITION = Radians.of(4.70).in(Radians);
    public static final double ALGAE_POSITION = Radians.of(4.91).in(Radians);
    /**
     * The position that aligns the arm with the coral station in radians.
     * This is the actual position minus a fudge factor that accounts for the weight of gravity.
     */
    public static final double CORAL_STATION_POSITION =
            Radians.of(4.022).minus(Radians.of(0.1)).in(Radians);
}
