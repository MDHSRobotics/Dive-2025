package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    private ElevatorConstants() {}

    public static final int ELEVATOR_ID = 15;
    public static final int ARM_ID = 4;
    public static final int WHEELS_ID = 5;

    /**
     * The current limit for the arm and wheels in amps.
     * This is currently set to the value suggested by
     * <a href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV for the NEO Vortex.</a>
     */
    public static final int CURRENT_LIMIT = 80;

    /** Units: radians */
    public static final double ARM_MIN_LIMIT = 2.22;
    /** Units: radians */
    public static final double ARM_MAX_LIMIT = 5.80;

    /**
     * Gear reduction of the elevator motor.
     */
    private static final double ELEVATOR_GEAR_RATIO = 20.0;

    /**
     * The distance from the center of the hex bar to the outer edge of the elevator chain.
     */
    private static final Distance ELEVATOR_GEAR_RADIUS = Centimeters.of(2.65);

    /**
     * The conversion of motor rotations to elevator chain rotations.
     * We multiply by 2 because the third stage moves twice the distance of the second stage.
     */
    public static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = ELEVATOR_GEAR_RATIO;

    /**
     * The conversion of elevator chain rotations to elevator distance traveled.
     * We don't account for the gear ratio here because it has already been accounted for.
     * We multiply by 2 a second time because the third stage moves twice as much as the second stage.
     * @see <a href="https://pro.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters">How we convert from rotations to meters</a>
     */
    public static final double ELEVATOR_ROTATIONS_TO_METERS_CONVERSION =
            2.0 * Math.PI * ELEVATOR_GEAR_RADIUS.in(Meters) * 2.0;

    /**
     * The conversion of motor input rotations to arm output radians.
     */
    public static final double ARM_POSITION_CONVERSION_FACTOR = Rotations.one().in(Radians);
    /**
     * The conversion of motor input rotations per minute to arm output radians per second.
     */
    public static final double ARM_VELOCITY_CONVERSION_FACTOR =
            Rotations.per(Minute).one().in(RadiansPerSecond);

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
    public static final double TROUGH_POSITION = 5.32;
    /** Units: radians */
    public static final double L2_POSITION = 4.562;
    /** Units: radians */
    public static final double ALGAE_POSITION = 4.91;
    /**
     * The position that aligns the arm with the coral station in radians.
     * This is the actual position minus a fudge factor that accounts for the weight of gravity.
     */
    public static final double CORAL_STATION_POSITION =
            Radians.of(4.022).minus(Radians.of(0.1)).minus(Degrees.of(1)).in(Radians);
    /** Units: radians */
    public static final double UP_POSITION = 3.385;
}
