package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.drive.requests.ProfiledSwerveRequest;
import java.util.EnumSet;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveTranslation m_slipCurrentCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();

    /** SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

    /** SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    /**
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /** SysId routine for characterizing slip current.
     * You must log the data yourself while the test is running.
     * @see <a href="https://pro.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html#preventing-wheel-slip">explanation on what to log</a>
     */
    private final SysIdRoutine m_sysIdRoutineSlipCurrent = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.1).per(Second),
                    Volts.of(0), // Dynamic should not be used for this routine
                    null, // Use default timeout (10 s)
                    state -> {}), // You must log the data yourself.
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_slipCurrentCharacterization.withVolts(output)), null, this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /* NetworkTables logging */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    /**
     * Provides the front limelight's bot pose estimate to the drivetrain.
     * The LimelightHelpers equivalent to this is {@link frc.robot.util.LimelightHelpers#getBotPoseEstimate_wpiBlue_MegaTag2(String) getBotPoseEstimate_wpiBlue_MegaTag2()}.
     */
    private final DoubleArraySubscriber frontPoseEstimateSub = inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME)
            .getDoubleArrayTopic("botpose_orb_wpiblue")
            .subscribe(null);
    /**
     * Provides the back limelight's bot pose estimate to the drivetrain.
     * The LimelightHelpers equivalent to this is {@link frc.robot.util.LimelightHelpers#getBotPoseEstimate_wpiBlue_MegaTag2(String) getBotPoseEstimate_wpiBlue_MegaTag2()}.
     */
    private final DoubleArraySubscriber backPoseEstimateSub = inst.getTable(VisionConstants.BACK_LIMELIGHT_NAME)
            .getDoubleArrayTopic("botpose_orb_wpiblue")
            .subscribe(null);

    /**
     * This NetworkTable is used to display driving information to AdvantageScope.
     * Open <a href="https://docs.advantagescope.org/tab-reference/odometry">the AdvantageScope docs</a> to see what this looks like.
     */
    private final NetworkTable stateTable = inst.getTable("DriveState");
    /** Logs the front bot pose estimate to AdvantageScope. */
    private final StructPublisher<Pose2d> frontPoseEstimatePub =
            stateTable.getStructTopic("Front Pose Estimate", Pose2d.struct).publish();
    /** Logs the back bot pose estimate to AdvantageScope. */
    private final StructPublisher<Pose2d> backPoseEstimatePub =
            stateTable.getStructTopic("Back Pose Estimate", Pose2d.struct).publish();
    /** Logs the tags that are currently visible from the front to AdvantageScope. */
    private final StructArrayPublisher<Translation3d> frontVisibleTagsPub = stateTable
            .getStructArrayTopic("Front Visible Tags", Translation3d.struct)
            .publish();
    /** Logs the tags that are currently visible from the back to AdvantageScope. */
    private final StructArrayPublisher<Translation3d> backVisibleTagsPub = stateTable
            .getStructArrayTopic("Back Visible Tags", Translation3d.struct)
            .publish();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        /*
         * Set the vision measurement std devs.
         * These are defaults from https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-wpilibs-pose-estimator
         */
        setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        // registerPoseEstimateListeners();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        /*
         * Set the vision measurement std devs.
         * These are defaults from https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-wpilibs-pose-estimator
         */
        setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        // registerPoseEstimateListeners();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        /*
         * Set the vision measurement std devs.
         * These are defaults from https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-wpilibs-pose-estimator
         */
        setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        // registerPoseEstimateListeners();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
                () -> getState().Pose, // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(pathApplyRobotSpeeds
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                        // PID constants for translation
                        DriveConstants.TRANSLATION_PID,
                        // PID constants for rotation
                        DriveConstants.ROTATION_PID),
                DriveConstants.PATHPLANNER_CONFIG,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
                );
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return this.run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Returns a command that resets the swerve request's motion profile,
     * and applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyProfiledRequest(Supplier<ProfiledSwerveRequest> requestSupplier) {
        return this.startRun(() -> requestSupplier.get().resetProfile(), () -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Registers the limelight pose listeners,
     * so the poses can be used as soon as they are recieved from the limelights.
     * @see frc.robot.util.LimelightHelpers#getBotPoseEstimate(String, String, boolean) the reference code for processing the input from this NetworkTable entry
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/networktables/listening-for-change.html">the explanation for listeners</a>
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">the limelight NetworkTables API</a> (look for botpose_orb_wpiblue)
     */
    private void registerPoseEstimateListeners() {
        inst.addListener(frontPoseEstimateSub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            NetworkTableValue value = event.valueData.value;
            double[] poseArray = value.getDoubleArray();
            // If there is no data available, don't log anything
            if (poseArray.length < 11) {
                DriverStation.reportWarning("An empty bot pose estimate was received from the front limelight.", false);
                frontVisibleTagsPub.set(FieldConstants.NO_VISIBLE_TAGS);
                return;
            }

            /* Get bot pose estimate */
            Translation2d botPose = new Translation2d(poseArray[0], poseArray[1]);
            Rotation2d botRotation = Rotation2d.fromDegrees(poseArray[5]);
            Pose2d botPoseEstimate = new Pose2d(botPose, botRotation);

            /* Get timestamp */
            long timestamp = value.getTime();

            /* Log pose estimate to AdvantageScope */
            frontPoseEstimatePub.set(botPoseEstimate, timestamp);

            // Convert timestamp from microseconds to seconds and adjust for latency
            double latency = poseArray[6];
            double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

            /* Add the vision measurement to the pose estimator */
            this.addVisionMeasurement(botPoseEstimate, Utils.fpgaToCurrentTime(adjustedTimestamp));

            /* Log which apriltags are currently visible */
            int tagCount = (int) poseArray[7];
            int valsPerFiducial = 7;
            int expectedTotalVals = 11 + valsPerFiducial * tagCount;

            // If there is no more data available, stop logging
            if (poseArray.length != expectedTotalVals || tagCount == 0) {
                DriverStation.reportWarning(
                        "There are no valid tags reported by the front limelight pose estimate.", false);
                frontVisibleTagsPub.set(FieldConstants.NO_VISIBLE_TAGS);
                return;
            }

            Translation3d[] visibleTagPositions = new Translation3d[tagCount];
            for (int i = 0; i < tagCount; i++) {
                int currentIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[currentIndex];
                visibleTagPositions[i] = FieldConstants.APRILTAG_POSES[id - 1];
            }
            frontVisibleTagsPub.set(visibleTagPositions, timestamp);
        });

        inst.addListener(backPoseEstimateSub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            NetworkTableValue value = event.valueData.value;
            double[] poseArray = value.getDoubleArray();
            // If there is no data available, don't log anything
            if (poseArray.length < 11) {
                DriverStation.reportWarning("An empty bot pose estimate was received from the back limelight.", false);
                backVisibleTagsPub.set(FieldConstants.NO_VISIBLE_TAGS);
                return;
            }

            /* Get bot pose estimate */
            Translation2d botPose = new Translation2d(poseArray[0], poseArray[1]);
            Rotation2d botRotation = Rotation2d.fromDegrees(poseArray[5]);
            Pose2d botPoseEstimate = new Pose2d(botPose, botRotation);

            /* Get timestamp */
            long timestamp = value.getTime();

            /* Log pose estimate to AdvantageScope */
            backPoseEstimatePub.set(botPoseEstimate, timestamp);

            // Convert timestamp from microseconds to seconds and adjust for latency
            double latency = poseArray[6];
            double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

            /* Add the vision measurement to the pose estimator */
            this.addVisionMeasurement(botPoseEstimate, Utils.fpgaToCurrentTime(adjustedTimestamp));

            /* Log which apriltags are currently visible */
            int tagCount = (int) poseArray[7];
            int valsPerFiducial = 7;
            int expectedTotalVals = 11 + valsPerFiducial * tagCount;

            // If there is no more data available, stop logging
            if (poseArray.length != expectedTotalVals || tagCount == 0) {
                DriverStation.reportWarning(
                        "There are no valid tags reported by the back limelight pose estimate.", false);
                backVisibleTagsPub.set(FieldConstants.NO_VISIBLE_TAGS);
                return;
            }

            Translation3d[] visibleTagPositions = new Translation3d[tagCount];
            for (int i = 0; i < tagCount; i++) {
                int currentIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[currentIndex];
                visibleTagPositions[i] = FieldConstants.APRILTAG_POSES[id - 1];
            }
            backVisibleTagsPub.set(visibleTagPositions, timestamp);
        });
    }
}
