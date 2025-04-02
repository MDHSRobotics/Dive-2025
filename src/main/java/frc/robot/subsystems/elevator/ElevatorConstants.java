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

    /** Units: rotations */
    public static final double ELEVATOR_MIN_LIMIT = 0;
    /** Units: rotations */
    public static final double ELEVATOR_MAX_LIMIT = 4.85;
    /** Units: radians */
    public static final double ARM_MIN_LIMIT = 3.911;
    /** Units: radians */
    public static final double ARM_MAX_LIMIT = 6.062;

    /**
     * Gear reduction of the elevator motor.
     */
    private static final double ELEVATOR_GEAR_REDUCTION = 12.0;

    /**
     * The distance from the center of the hex bar to the outer edge of the elevator chain.
     */
    private static final Distance ELEVATOR_GEAR_RADIUS = Centimeters.of(2.65);

    /**
     * The conversion of motor rotations to elevator chain rotations.
     */
    public static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = ELEVATOR_GEAR_REDUCTION;

    /**
     * The conversion of elevator chain rotations to elevator distance traveled.
     * We don't account for the gear ratio here because it has already been accounted for.
     * @see <a href="https://pro.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters">How we convert from rotations to meters</a>
     */
    public static final double ELEVATOR_ROTATIONS_TO_METERS_CONVERSION =
            2.0 * Math.PI * ELEVATOR_GEAR_RADIUS.in(Meters);

    /** Units: volts */
    public static final double ELEVATOR_K_G = 0.18892;
    /** Units: volts */
    public static final double ELEVATOR_K_S = 0.074155;
    /** Units: volts per rotation/sec */
    public static final double ELEVATOR_K_V = 3.172;
    /** Units: volts per rotation/sec^2 */
    public static final double ELEVATOR_K_A = 0.030076;

    /** Units: volts per rotation */
    public static final double ELEVATOR_K_P = 0;

    // Common elevator positions
    /** Units: rotations */
    public static final double ELEVATOR_STOWED_POSITION = ELEVATOR_MIN_LIMIT;
    /** Units: rotations */
    public static final double ELEVATOR_CORAL_STATION_POSITION = ELEVATOR_MAX_LIMIT;
    /** Units: rotations */
    public static final double ELEVATOR_L1_POSITION = 0.1230;
    /** Units: rotations */
    public static final double ELEVATOR_L2_POSITION = 0.9212;
    /** Units: rotations */
    public static final double ELEVATOR_L3_POSITION = 2.1499;

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
    public static final double ARM_ZERO_OFFSET = 0;

    /** The position of the absolute encoder in radians that makes the arm stick straight out at the horizontal. */
    public static final double ARM_HORIZONTAL_OFFSET = 5.162;

    /** Units: volts */
    public static final double ARM_K_G = 1;
    /** Units: volts */
    public static final double ARM_K_S = 0.22457;
    /** Units: volts per rad/sec */
    public static final double ARM_K_V = 0.13233;
    /** Units: volts per rad/sec^2 */
    public static final double ARM_K_A = 0.039128;

    /** Units: rad/sec */
    public static final double ARM_MAX_VELOCITY = 6.587;
    /** Units: rad/sec^2 */
    public static final double ARM_MAX_ACCELERATION = 55.5;

    /** Units: duty cycle out per radian */
    public static final double ARM_K_P = 0.5;

    // Common arm positions
    /** Units: radians */
    public static final double ARM_STOWED_POSITION = ARM_MIN_LIMIT;
    /** Units: radians */
    public static final double ARM_CORAL_STATION_POSITION = ARM_MAX_LIMIT;
    /** Units: radians */
    public static final double ARM_L1_POSITION = 5.107;
    /** Units: radians */
    public static final double ARM_L2_AND_L3_POSITION = 4.830;
}
