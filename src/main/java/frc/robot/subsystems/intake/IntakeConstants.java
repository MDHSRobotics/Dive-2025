package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

public class IntakeConstants {
    private IntakeConstants() {}

    public static final int ARM_ID = 6;
    public static final int WHEEL_RIGHT_ID = 7;
    public static final int WHEEL_LEFT_ID = 8;

    /**
     * The roboRIO DIO channel that the
     * <a href="https://www.adafruit.com/product/2168">beam break sensor</a>
     * is plugged into.
     */
    public static final int ARM_BEAM_BEAK_DIO_CHANNEL = 8;

    /**
     * The current limit for the arm in amps.
     * This is currently set to the value suggested by
     * <a href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV for the NEO Vortex.</a>
     */
    public static final int ARM_CURRENT_LIMIT = 80;
    /**
     * The current limit for the wheels in amps.
     * This is currently set to the value suggested by
     * <a href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV for the NEO 550.</a>
     */
    public static final int WHEEL_CURRENT_LIMIT = 40;

    /** Units: radians */
    public static final double ARM_MAX_LIMIT = 5.411;
    /** Units: radians */
    public static final double ARM_MIN_LIMIT = 3.124;

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
    public static final double ARM_ZERO_OFFSET = 0.25;

    /**
     * The conversion of motor input rotations to wheel output radians.
     * Motor input rotations will be divided by the gear ratio, and then converted to radians.
     */
    public static final double WHEEL_POSITION_CONVERSION_FACTOR =
            Rotations.one().div(4).in(Radians);
    /**
     * The conversion of motor input rotations per minute to wheel output radians per second.
     * Motor input rotations per minute will divided by the gear ratio, and converted to radians per second.
     */
    public static final double WHEEL_VELOCITY_CONVERSION_FACTOR =
            Rotations.per(Minute).one().div(4.0).in(RadiansPerSecond);

    /**
     * Proportional gain for the <a href="https://docs.revrobotics.com/revlib/spark/closed-loop#closed-loop-control-with-spark-motor-controllers">internal closed loop controller</a>
     * in duty cycle (percent out) per radian.
     * This still needs to be tuned.
     */
    public static final double K_P = 0.2;

    // Common intake positions
    /**
     * The position that aligns the arm with the algae resting on a coral in radians.
     */
    public static final double ON_CORAL_PICKUP_POSITION = 3.854;
    /**
     * The position that aligns the arm with the algae resting on the ground in radians.
     */
    public static final double GROUND_PICKUP_POSITION = 3.155;
    /** Units: radians */
    public static final double PROCESSOR_POSITION = 3.621;
}
