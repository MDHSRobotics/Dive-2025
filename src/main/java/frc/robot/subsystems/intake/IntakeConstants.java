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
    public static final int ARM_CURRENT_LIMIT = (int) Amps.of(80).in(Amps);
    /**
     * The current limit for the wheels in amps.
     * This is currently set to the value suggested by
     * <a href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV for the NEO 550.</a>
     */
    public static final int WHEEL_CURRENT_LIMIT = (int) Amps.of(40).in(Amps);

    public static final double ARM_MAX_LIMIT = Radians.of(4.424).in(Radians);
    public static final double ARM_MIN_LIMIT = Radians.of(2.137).in(Radians);

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
    public static final double ARM_ZERO_OFFSET = 0.25;

    /**
     * The conversion of motor input rotations to wheel output radians.
     * Motor input rotations will be divided by the gear ratio, and then converted to radians.
     */
    public static final double WHEEL_POSITION_CONVERSION_FACTOR =
            Rotations.of(1).div(4).in(Radians);
    /**
     * The conversion of motor input rotations per minute to wheel output radians per second.
     * Motor input rotations per minute will divided by the gear ratio, and converted to radians per second.
     */
    public static final double WHEEL_VELOCITY_CONVERSION_FACTOR =
            Rotations.per(Minute).of(1).div(4.0).in(RadiansPerSecond);

    /**
     * Proportional gain for the <a href="https://docs.revrobotics.com/revlib/spark/closed-loop#closed-loop-control-with-spark-motor-controllers">internal closed loop controller</a>
     * in duty cycle (percent out) per radian.
     * This still needs to be tuned.
     */
    public static final double K_P = 0.3;

    // Common intake positions
    /**
     * The position that aligns the arm with the algae resting on a coral.
     * This still needs to be found.
     */
    public static final double ON_CORAL_PICKUP_POSITION = Radians.of(2.867).in(Radians);
    /**
     * The position that aligns the arm with the algae resting on the ground.
     * This still needs to be found.
     */
    public static final double GROUND_PICKUP_POSITION = Radians.of(2.168).in(Radians);

    public static final double PROCESSOR_POSITION = Radians.of(2.634).in(Radians);
}
