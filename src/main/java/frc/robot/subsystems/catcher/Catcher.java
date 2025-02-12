package frc.robot.subsystems.catcher;

import static frc.robot.Constants.K_DT;
import static frc.robot.subsystems.catcher.CatcherConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

public class Catcher extends SubsystemBase {
    public enum CatcherArmPositions {
        TROUGH,
        CORAL_STATION
    }

    private final SparkFlex m_armMotor = new SparkFlex(ARM_ID, MotorType.kBrushless);
    private final SparkFlex m_flywheelsMotor = new SparkFlex(WHEELS_ID, MotorType.kBrushless);

    private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    // private final DigitalInput m_armBeamBreak = new DigitalInput(9);

    private final SysIdRoutine m_armRoutine =
            new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(m_armMotor::setVoltage, null, this));

    private final SimpleMotorFeedforward m_armFeedforward = new SimpleMotorFeedforward(K_S, K_V, K_A);

    private final TrapezoidProfile m_armProfile = new TrapezoidProfile(ARM_ANGULAR_MOTION_CONSTRAINTS);
    private final TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_previousSetpoint = new TrapezoidProfile.State();

    private final SparkClosedLoopController m_armController = m_armMotor.getClosedLoopController();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Catcher");
    private final DoubleEntry flywheelSpeedEntry =
            table.getDoubleTopic("Flywheel Speed").getEntry(1);

    /**
     * Motors should be configured in the robot code rather than the REV Hardware Client
     * so that we can see the motor configs without having to connect to the robot.
     * For this reason, values set in the REV Hardware Client will be cleared when this constructor runs.
     */
    public Catcher() {
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake);
        armConfig
                .encoder
                .positionConversionFactor(ARM_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(ARM_VELOCITY_CONVERSION_FACTOR);
        armConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(K_P)
                .d(K_D);
        armConfig
                .signals
                .primaryEncoderPositionPeriodMs(10)
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(10)
                .primaryEncoderVelocityAlwaysOn(true);
        m_armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig flywheelsConfig = new SparkFlexConfig();
        flywheelsConfig
                .smartCurrentLimit(CURRENT_LIMIT)
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        m_flywheelsMotor.configure(flywheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // You need to publish a value for the entry to appear in NetworkTables
        flywheelSpeedEntry.set(1);
    }

    private void resetProfile(CatcherArmPositions armPosition) {
        if (armPosition == CatcherArmPositions.TROUGH) {
            m_goal.position = TROUGH_POSITION;
        } else {
            m_goal.position = CORAL_STATION_POSITION;
        }
        m_previousSetpoint.position = m_armEncoder.getPosition();
        m_previousSetpoint.velocity = m_armEncoder.getVelocity();
    }

    private void runProfile() {
        // Find the next position in the motion
        TrapezoidProfile.State nextSetpoint = m_armProfile.calculate(K_DT, m_previousSetpoint, m_goal);
        // Estimate the volts required to reach the position
        double feedforwardVolts =
                m_armFeedforward.calculateWithVelocities(m_previousSetpoint.velocity, nextSetpoint.velocity);
        // Run the PID controller along with the estimated volts
        m_armController.setReference(
                nextSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts);
        // Update current setpoint
        m_previousSetpoint = nextSetpoint;
    }

    public Command disableMotorsCommand() {
        return this.runOnce(() -> {
                    m_armMotor.stopMotor();
                    m_flywheelsMotor.stopMotor();
                })
                .andThen(Commands.idle(this));
    }

    public Command armTestCommand(DoubleSupplier armPowerSupplier) {
        return this.run(() -> {
            m_armMotor.set(armPowerSupplier.getAsDouble() * 0.25);
        });
    }

    public Command wheelTestCommand() {
        return this.run(() -> m_flywheelsMotor.set(flywheelSpeedEntry.get() * 0.25));
    }

    public Command wheelBackwardsTestCommand() {
        return this.run(() -> m_flywheelsMotor.set(-flywheelSpeedEntry.get() * 0.5));
    }

    public Command setArmPositionCommand(CatcherArmPositions armPosition) {
        return this.startRun(() -> this.resetProfile(armPosition), this::runProfile);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_armRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_armRoutine.dynamic(direction);
    }
}
