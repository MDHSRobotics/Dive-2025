package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.TunerConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;

/**
 * This class contains a command that spins the robot to find the radius of the wheels.
 * <p>
 * This command was taken from
 * <a href="https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/talonfx_swerve/src/main/java/frc/robot/commands/DriveCommands.java#L223">
 * AdvantageKit's example project</a>,
 * and modified to work with a Phoenix generated swerve.
 * @see <a href="https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#wheel-radius-characterization">AdvantageKit's explanation on how to use the command</a>
 */
public class WheelRadiusCharacterization {
    private WheelRadiusCharacterization() {}

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    private static final double WHEEL_RADIUS_MAX_VELOCITY =
            RadiansPerSecond.of(0.25).in(RadiansPerSecond);
    private static final double WHEEL_RADIUS_RAMP_RATE =
            RadiansPerSecondPerSecond.of(0.05).in(RadiansPerSecondPerSecond);

    public static Command characterizationCommand(CommandSwerveDrivetrain drivetrain) {
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = drivetrain.getModules();
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
        SwerveRequest.ApplyRobotSpeeds chassisSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> {
                            limiter.reset(0.0);
                        }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drivetrain.setControl(
                                            chassisSpeedsRequest.withSpeeds(new ChassisSpeeds(0, 0, speed)));
                                },
                                drivetrain)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(() -> {
                            for (int i = 0; i < 4; ++i) {
                                state.positions[i] = Units.rotationsToRadians(
                                        modules[i].getDriveMotor().getPosition().getValueAsDouble()
                                                / TunerConstants.kDriveGearRatio);
                            }
                            SwerveDriveState currentState = drivetrain.getState();
                            state.lastAngle = currentState.RawHeading;
                            state.gyroDelta = 0.0;
                        }),

                        // Update gyro delta
                        Commands.run(() -> {
                                    Rotation2d rotation = drivetrain.getState().RawHeading;
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    double[] positions = new double[4];
                                    for (int i = 0; i < 4; ++i) {
                                        positions[i] = Units.rotationsToRadians(modules[i]
                                                        .getDriveMotor()
                                                        .getPosition()
                                                        .getValueAsDouble()
                                                / TunerConstants.kDriveGearRatio);
                                    }
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius =
                                            (state.gyroDelta * DriveConstants.DRIVEBASE_RADIUS.in(Meters)) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters, "
                                            + formatter.format(
                                                    edu.wpi.first.math.util.Units.metersToInches(wheelRadius))
                                            + " inches");
                                })));
    }
}
