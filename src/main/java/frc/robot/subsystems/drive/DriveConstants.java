package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;

/**
 * Contains constants for the swerve drive that haven't been specified in {@link frc.robot.subsystems.drive.TunerConstants TunerConstants}.
 * <p>
 * Some constants are pre-converted to avoid repeated unit conversions in the main robot loop.
 */
public class DriveConstants {
    private DriveConstants() {}

    /**
     * Distance between front left module (cancoder) and front right module (cancoder)
     */
    private static final Distance TRACKWIDTH = Inches.of(20.25);
    /**
     * Distance between front left module (cancoder) and back left module (cancoder)
     */
    private static final Distance WHEELBASE = Inches.of(25.25);
    /**
     * Distance from center of robot to a module (cancoder)
     */
    public static final Distance DRIVEBASE_RADIUS = Inches.of(
            Math.hypot(TRACKWIDTH.div(2.0).in(Inches), WHEELBASE.div(2.0).in(Inches)));

    /** Max linear speed of the robot in meters per second. This still needs to be tuned. */
    public static final double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /**
     * Max linear acceleration of the robot.
     */
    public static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(60);

    /**
     * Constraints for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">motion profiles</a> used in custom swerve requests.
     * This still needs to be tuned.
     */
    public static final TrapezoidProfile.Constraints LINEAR_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_LINEAR_SPEED / 2.0, MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond) / 2.0);

    /**
     * Proportional gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">x and y PID controllers</a>
     * used in custom swerve requests.
     * The gain is output linear velocity (meters per second) per error (meters).
     * This still needs to be tuned.
     */
    public static final double K_TRANSLATION_P =
            MetersPerSecond.per(Meter).ofNative(0).in(MetersPerSecond.per(Meter));

    /**
     * Derivative gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">x and y PID controllers</a>
     * used in custom swerve requests.
     * The gain is output linear velocity (meters per second) per the rate of change of error (meters per second).
     * This still needs to be tuned.
     */
    public static final double K_TRANSLATION_D =
            MetersPerSecond.per(MetersPerSecond).ofNative(0).in(MetersPerSecond.per(MetersPerSecond));

    /**
     * Max angular rate of the robot in radians per second.
     * @see <a href="https://math.libretexts.org/Bookshelves/Precalculus/Elementary_Trigonometry_(Corral)/04%3A_Radian_Measure/4.04%3A_Circular_Motion-_Linear_and_Angular_Speed">explanation on how to convert from linear velocity to angular velocity</a>
     */
    public static final double MAX_ANGULAR_RATE =
            RadiansPerSecond.of(MAX_LINEAR_SPEED / DRIVEBASE_RADIUS.in(Meters)).in(RadiansPerSecond);

    /**
     * Max angular acceleration of the robot.
     */
    private static final AngularAcceleration MAX_ANGULAR_ACCEL = RadiansPerSecondPerSecond.of(75.215);

    /**
     * Constraints for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">motion profiles</a> used in custom swerve requests.
     * This still needs to be tuned.
     */
    public static final TrapezoidProfile.Constraints ANGULAR_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGULAR_RATE / 2.0, MAX_ANGULAR_ACCEL.in(RadiansPerSecondPerSecond) / 2.0);

    /**
     * Proportional gain for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">heading PID controller</a>
     * used in custom swerve requests.
     * The gain is output angular velocity (radians per second) per error (radians).
     * This comes from "Rotation RotationalRate Position.PNG"
     */
    public static final double K_ANGULAR_P =
            RadiansPerSecond.per(Radian).ofNative(4).in(RadiansPerSecond.per(Radian));

    /**
     * Goal tolerance for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">heading PID controller</a>
     * used in custom swerve requests.
     * This still needs to be tuned.
     */
    public static final Angle GOAL_TOLERANCE = Degrees.of(1);

    /**
     * Multiply wheel rotations by this number to convert to meters.
     * This accounts for the drive gear ratio.
     * Units: meter / rotations
     * @see <a href="https://pro.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters">How we convert from rotations to meters</a>
     */
    private static final double WHEEL_ROTATIONS_TO_METERS_CONVERSION =
            2.0 * Math.PI * TunerConstants.kWheelRadius.in(Meters) / TunerConstants.kDriveGearRatio;

    /* PathPlanner Configuration */

    /**
     * PID Constants for PathPlanner translation.
     * This still needs to be tuned.
     */
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);

    /**
     * PID Constants for PathPlanner rotation.
     * This still needs to be tuned.
     */
    public static final PIDConstants ROTATION_PID = new PIDConstants(7, 0, 0);

    /**
     * Robot mass with battery.
     * Last measured value: 105 pounds-force, coverted to kg.
     */
    private static final Mass ROBOT_MASS = Kilograms.of(47.6272);

    /**
     * Angular acceleration gain from {@link com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation a SysId routine}.
     * This comes from the Rotation Voltage Position/Velocity PNGs in the project files.
     */
    private static final Per<VoltageUnit, AngularAccelerationUnit> K_A_ANGULAR =
            Volts.per(DegreesPerSecondPerSecond).ofNative(0.0015539);

    /**
     * Linear acceleration gain from {@link com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation a SysId routine}.
     * This comes from "Translation Velocity.PNG" in the project files.
     * 0.0024069 volts/rotation per second squared / 0.0473146029 meters/rotation
     */
    private static final Per<VoltageUnit, LinearAccelerationUnit> K_A_LINEAR =
            VoltsPerMeterPerSecondSquared.ofNative(0.0024069 / WHEEL_ROTATIONS_TO_METERS_CONVERSION);

    /**
     * The robot's moment of inertia.
     * I'm not sure if the units are actually KG*M^2, but this how PathPlanner says to calculate it.
     * @see <a href="https://pathplanner.dev/robot-config.html#calculating-moi-through-sysid-recommended">the equation for calculating MOI through SysId</a>
     * @see <a href="https://choreo.autos/usage/estimating-moi/">the equation, but with some units specified</a>
     * @see <a href="https://www.chiefdelphi.com/t/question-about-calculating-moi-with-sysid/490893">why we use drivebase radius instead of trackwidth / 2</a>
     */
    private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(ROBOT_MASS.in(Kilograms)
            * DRIVEBASE_RADIUS.in(Meters)
            * (K_A_ANGULAR.in(VoltsPerRadianPerSecondSquared) / K_A_LINEAR.in(VoltsPerMeterPerSecondSquared)));

    /** Wheel coefficient of friction for <a href="https://www.vexrobotics.com/colsonperforma.html">Colson wheels.</a> */
    private static final double WHEEL_COF = 1.0;

    /** The swerve module config to be used for every module. */
    private static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
            TunerConstants.kWheelRadius,
            TunerConstants.kSpeedAt12Volts,
            WHEEL_COF,
            DCMotor.getKrakenX60(1),
            TunerConstants.kDriveGearRatio,
            TunerConstants.kSlipCurrent,
            1);

    /**
     * The locations of the modules relative to the center of the robot.
     * The order is FL, FR, BL, and BR.
     */
    private static final Translation2d[] MODULE_OFFSETS = new Translation2d[] {
        new Translation2d(TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos),
        new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos),
        new Translation2d(TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos),
        new Translation2d(TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos)
    };

    public static final RobotConfig PATHPLANNER_CONFIG =
            new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG, MODULE_OFFSETS);
}
