package frc.robot.subsystems.catcher;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

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

    /**
     * The conversion of motor input rotations to arm output radians.
     * Motor input rotations will be divided by the gear ratio, and then converted to radians.
     */
    public static final double ARM_POSITION_CONVERSION_FACTOR =
            Rotations.of(1).div(49.5).in(Radians);
    /**
     * The conversion of motor input rotations per minute to arm output radians per second.
     * Motor input rotations per minute will divided by the gear ratio, and converted to radians per second.
     */
    public static final double ARM_VELOCITY_CONVERSION_FACTOR =
            Rotations.per(Minute).of(1).div(49.5).in(RadiansPerSecond);

    /**
     * Static gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#arm-feedforward">arm feedforward</a>
     * in volts.
     * This still needs to be tuned.
     */
    public static final double K_S = Volts.of(0).in(Volts);
    /**
     * Gravity gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#arm-feedforward">arm feedforward</a>
     * in volts.
     * This still needs to be tuned.
     */
    public static final double K_G = Volts.of(0).in(Volts);
    /**
     * Velocity gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#arm-feedforward">arm feedforward</a>
     * in volts per radian per second.
     * This still needs to be tuned.
     */
    public static final double K_V = VoltsPerRadianPerSecond.ofNative(0).in(VoltsPerRadianPerSecond);
    /**
     * Acceleration gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#arm-feedforward">arm feedforward</a>
     * in volts per radian per second squared.
     * This still needs to be tuned.
     */
    public static final double K_A = VoltsPerRadianPerSecondSquared.ofNative(0).in(VoltsPerRadianPerSecondSquared);

    /**
     * Proportional gain for the <a href="https://docs.revrobotics.com/revlib/spark/closed-loop#closed-loop-control-with-spark-motor-controllers">internal closed loop controller</a>
     * in volts per radian.
     * This still needs to be tuned.
     */
    public static final double K_P = Volts.per(Radian).ofNative(0).in(Volts.per(Radian));
    /**
     * Derivative gain for the <a href="https://docs.revrobotics.com/revlib/spark/closed-loop#closed-loop-control-with-spark-motor-controllers">internal closed loop controller</a>
     * in volts per radian per second.
     * This still needs to be tuned.
     */
    public static final double K_D = VoltsPerRadianPerSecond.ofNative(0).in(VoltsPerRadianPerSecond);

    /**
     * Maximum allowed angular velocity for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">motion profile</a>
     * This still needs to be tuned.
     */
    private static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(0);
    /**
     * Maximum allowed angular acceleration for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">motion profile</a>
     * This still needs to be tuned.
     */
    private static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(0);

    /**
     * Maximum allowed angular velocity (in radians per second) and acceleration (in radians per second per second).
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">explanation for motion profiles</a>
     */
    public static final TrapezoidProfile.Constraints ARM_ANGULAR_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond));

    // Common catcher positions
    /**
     * The position that aligns the arm with the trough in radians.
     * This still needs to be found.
     */
    public static final double TROUGH_POSITION = Radians.of(0).in(Radians);
    /**
     * The position that aligns the arm with the coral station in radians.
     * This still needs to be found.
     */
    public static final double CORAL_STATION_POSITION = Radians.of(0).in(Radians);
}
