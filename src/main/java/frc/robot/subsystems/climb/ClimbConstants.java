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
    public static final double BACK_MAX_LIMIT = 4.986;
    /** The angle that pushes the cage down as far as possible in radians. */
    public static final double BACK_MIN_LIMIT = 2.032;

    /** The furthest safe angle that is away from the cage in radians. */
    public static final double FRONT_MAX_LIMIT = 2.97;
    /** The angle that pushes the cage down as far as possible in radians. */
    public static final double FRONT_MIN_LIMIT = 0.085;

    /**
     * The position of the absolute encoder (before any position conversion factor) that reports 0.
     * This is currently set outside the motor's range of motion to prevent the position wrapping around from 1 to 0.
     */
    public static final double BACK_ZERO_OFFSET = 0.5;

    /**
     * The position of the absolute encoder (before any position conversion factor) that reports 0.
     * This is currently set outside the motor's range of motion to prevent the position wrapping around from 1 to 0.
     */
    public static final double FRONT_ZERO_OFFSET = 0.75;

    /**
     * The conversion of motor input rotations to climb hook output radians.
     */
    public static final double POSITION_CONVERSION_FACTOR = Rotations.one().in(Radians);
    /**
     * The conversion of motor input rotations per minute to climb hook output radians per second.
     */
    public static final double VELOCITY_CONVERSION_FACTOR =
            Rotations.per(Minute).one().in(RadiansPerSecond);

    /**
     * Static gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html">feedforward</a>
     * in volts.
     * This still needs to be tuned.
     */
    // public static final double K_S = 0;
    /**
     * Velocity gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html">feedforward</a>
     * in volts per radian per second.
     * This still needs to be tuned.
     */
    // public static final double K_V = 0;
    /**
     * Acceleration gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html">feedforward</a>
     * in volts per radian per second squared.
     * This still needs to be tuned.
     */
    // public static final double K_A = 0;

    /**
     * Proportional gain for the <a href="https://docs.revrobotics.com/revlib/spark/closed-loop#closed-loop-control-with-spark-motor-controllers">internal closed loop controller</a>
     * in duty cycle (percent out) per radian.
     * This still needs to be tuned.
     */
    // public static final double K_P = 0.5;
    /**
     * Derivative gain for the <a href="https://docs.revrobotics.com/revlib/spark/closed-loop#closed-loop-control-with-spark-motor-controllers">internal closed loop controller</a>
     * in duty cycle (percent out) per radian per second.
     * This still needs to be tuned.
     */
    // public static final double K_D = 0;

    /**
     * Maximum allowed angular velocity for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">motion profile</a>
     * This still needs to be tuned.
     */
    // private static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(0);
    /**
     * Maximum allowed angular acceleration for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">motion profile</a>
     * This still needs to be tuned.
     */
    // private static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(0);

    /**
     * Maximum allowed angular velocity (in radians per second) and acceleration (in radians per second per second).
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">explanation for motion profiles</a>
     */
    // public static final TrapezoidProfile.Constraints ANGULAR_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(
    //         MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond));

    // Common climb positions
    /**
     * The position that keeps the claws out of the cage in radians.
     */
    public static final double FRONT_UP_POSITION = 3.244;
    /**
     * The position that keeps the claws out of the cage in radians.
     */
    public static final double BACK_UP_POSITION = 5.005;
}
