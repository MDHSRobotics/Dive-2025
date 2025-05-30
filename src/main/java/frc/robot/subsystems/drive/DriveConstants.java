package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
     * Distance between center to front bumper
     */
    public static final Distance CENTER_TO_FRONT_BUMPER_LENGTH = Inches.of(18);

    public static final Distance BUMPER_TO_BUMPER_X_DISTANCE = Inches.of(33);
    public static final Distance BUMPER_TO_BUMPER_Y_DISTANCE = Inches.of(36);

    /**
     * Distance between front left module (cancoder) and front right module (cancoder)
     */
    private static final Distance TRACKWIDTH = TunerConstants.kFrontLeftYPos.minus(TunerConstants.kFrontRightYPos);
    /**
     * Distance between front left module (cancoder) and back left module (cancoder)
     */
    private static final Distance WHEELBASE = TunerConstants.kFrontLeftXPos.minus(TunerConstants.kBackLeftXPos);
    /**
     * Distance from center of robot to a module (cancoder)
     */
    public static final Distance DRIVEBASE_RADIUS = Inches.of(
            Math.hypot(TRACKWIDTH.div(2.0).in(Inches), WHEELBASE.div(2.0).in(Inches)));

    /**
     * Max linear speed of the robot in meters per second.
     * In reality, it takes quite some time to reach this velocity due to the low drive slip current limit.
     */
    public static final double MAX_LINEAR_SPEED = MetersPerSecond.of(4).in(MetersPerSecond);

    /**
     * Constraints for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">motion profiles</a> used in custom swerve requests.
     */
    public static final TrapezoidProfile.Constraints LINEAR_MOTION_CONSTRAINTS =
            new TrapezoidProfile.Constraints(4.0, 2.0);

    /**
     * Max angular rate of the robot in radians per second.
     * @see <a href="https://math.libretexts.org/Bookshelves/Precalculus/Elementary_Trigonometry_(Corral)/04%3A_Radian_Measure/4.04%3A_Circular_Motion-_Linear_and_Angular_Speed">explanation on how to convert from linear velocity to angular velocity</a>
     */
    public static final double MAX_ANGULAR_RATE =
            RadiansPerSecond.of(MAX_LINEAR_SPEED / DRIVEBASE_RADIUS.in(Meters)).in(RadiansPerSecond);

    /**
     * Goal tolerance for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">heading PID controller</a>
     * used in custom swerve requests.
     */
    public static final Angle HEADING_TOLERANCE = Degrees.of(2);

    /**
     * Goal tolerance for the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">x and y PID controllers</a>
     * used in custom swerve requests.
     */
    public static final Distance LINEAR_TOLERANCE = Inches.of(0.5);

    /**
     * Multiply wheel rotations by this number to convert to meters.
     * This accounts for the drive gear ratio.
     * Units: meter / rotations
     * @see <a href="https://pro.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters">How we convert from rotations to meters</a>
     */
    private static final double WHEEL_ROTATIONS_TO_METERS_CONVERSION =
            2.0 * Math.PI * TunerConstants.kWheelRadius.in(Meters) / TunerConstants.kDriveGearRatio;

    /* PathPlanner Configuration
     * We configure PathPlanner here instead of in the GUI so we can use constants from the code.
     */

    /**
     * PID Constants for PathPlanner translation.
     */
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);

    /**
     * PID Constants for PathPlanner rotation.
     */
    public static final PIDConstants ROTATION_PID = new PIDConstants(5.0, 0.0, 0.0);

    /**
     * Robot mass with battery and bumpers.
     * Last measured value: 126 pounds-force, coverted to kg.
     */
    public static final Mass ROBOT_MASS = Kilograms.of(57.15264);

    /**
     * Angular acceleration gain from {@link com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation a SysId routine}.
     * This comes from the Rotation Voltage Position/Velocity PNGs in the project files.
     */
    private static final Per<VoltageUnit, AngularAccelerationUnit> K_A_ANGULAR =
            Volts.per(DegreesPerSecondPerSecond).ofNative(0.0015539);

    /**
     * Linear acceleration gain from {@link com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation a SysId routine}.
     * This comes from "Translation Velocity.PNG" in the project files.
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
    public static final double WHEEL_COF = 1.0;

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

    public static final PathConstraints ON_THE_FLY_CONSTRAINTS = new PathConstraints(
            LINEAR_MOTION_CONSTRAINTS.maxVelocity,
            LINEAR_MOTION_CONSTRAINTS.maxAcceleration,
            Units.degreesToRadians(540),
            Units.degreesToRadians(540),
            12);
    public static final PathConstraints CORAL_STATION_CONSTRAINTS =
            new PathConstraints(4, 4, Units.degreesToRadians(540), Units.degreesToRadians(540), 12);

    /* Swerve Setpoint Generator Constants */
    /**
     * The maximum angular velocity of the steer motor in radians per second.
     * <p>
     * We limit the voltage here to 7 V because the highest observed voltage is currently about 8 V.
     * <p>
     * Units: volts / volts per radian per second
     */
    public static final double MAX_STEER_VELOCITY = 7.0 / TunerConstants.steerGains.kV;
    /** This can safely be reused by multiple swerve requests because it has no internal state (as of FRC 2025). */
    public static final SwerveSetpointGenerator SWERVE_SETPOINT_GENERATOR =
            new SwerveSetpointGenerator(PATHPLANNER_CONFIG, MAX_ANGULAR_RATE);
}
