package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private static final String LIMELIGHT_NAME = "limelight-front";
    private static final Matrix<N3, N1> kVisionStdDevs =
        VecBuilder.fill(0.1, 0.1, 9999999.0);

    // Choreo follower PID
    private final PIDController m_xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController m_yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController m_headingController = new PIDController(7.5, 0.0, 0.0);

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
        new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
        new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds m_choreoApplyRobotSpeeds =
        new SwerveRequest.ApplyRobotSpeeds();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);

        m_headingController.enableContinuousInput(-Math.PI, Math.PI);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);

        m_headingController.enableContinuousInput(-Math.PI, Math.PI);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);

        m_headingController.enableContinuousInput(-Math.PI, Math.PI);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        updateVision();
    }

    /**
     * Choreo trajectory follower.
     * Feed this into AutoFactory.
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getState().Pose;
        Pose2d referencePose = sample.getPose();
        ChassisSpeeds ff = sample.getChassisSpeeds(); // field-relative

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
            ff.vxMetersPerSecond + m_xController.calculate(pose.getX(), referencePose.getX()),
            ff.vyMetersPerSecond + m_yController.calculate(pose.getY(), referencePose.getY()),
            ff.omegaRadiansPerSecond
                + m_headingController.calculate(
                    pose.getRotation().getRadians(),
                    referencePose.getRotation().getRadians()
                )
        );

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds,
            pose.getRotation()
        );

        setControl(m_choreoApplyRobotSpeeds.withSpeeds(robotRelativeSpeeds));
    }

    /**
     * Convenience helper so RobotContainer can grab a Choreo AutoFactory.
     */
    public AutoFactory createAutoFactory() {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followTrajectory,
            true,
            this
        );
    }

    private void updateVision() {
        double yawDeg = getState().Pose.getRotation().getDegrees();
        double gyroVelDegPerSec = getState().Speeds.omegaRadiansPerSecond * 180.0 / Math.PI;

        LimelightHelpers.SetRobotOrientation(
            LIMELIGHT_NAME,
            yawDeg,
            gyroVelDegPerSec,
            0.0,
            0.0,
            0.0,
            0.0
        );

        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        if (mt2 == null) {
            return;
        }

        boolean doRejectUpdate = false;

        if (Math.abs(gyroVelDegPerSec) > 200.0) {
            doRejectUpdate = true;
        }

        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (mt2.tagCount == 1 && mt2.rawFiducials != null && mt2.rawFiducials.length == 1) {
            if (mt2.rawFiducials[0].ambiguity > 0.7) {
                doRejectUpdate = true;
            }
            if (mt2.rawFiducials[0].distToCamera > 3.0) {
                doRejectUpdate = true;
            }
        }

        if (!doRejectUpdate) {
            addVisionMeasurement(mt2.pose, mt2.timestampSeconds, kVisionStdDevs);
        }

        SmartDashboard.putNumber("LL Tag Count", mt2.tagCount);
        SmartDashboard.putNumber("LL Pose X", mt2.pose.getX());
        SmartDashboard.putNumber("LL Pose Y", mt2.pose.getY());
        SmartDashboard.putNumber("LL Pose Rot", mt2.pose.getRotation().getDegrees());
        SmartDashboard.putNumber("LL Gyro Vel DegPerSec", gyroVelDegPerSec);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs
        );
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}