package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  public static class ShooterConstants {
    // Hardware
    public static final int kShooterMotorCAN = 20; // <-- change
    public static final boolean kMotorInverted = false;

    // Flywheel geometry
    public static final double kFlywheelDiameterMeters = 0.1016; // 4 in
    public static final double kFlywheelRadiusMeters = kFlywheelDiameterMeters / 2.0;

    // Gear ratio: motorRotations * kGearRatio = flywheelRotations
    public static final double kGearRatio = 1.0;

    // Exit speed ~= wheel surface speed * this
    public static final double kExitSpeedPerWheelSpeed = 0.85; // tune

    // Limits
    public static final double kMinRPM = 0;
    public static final double kMaxRPM = 6000; // tune safe max

    // Closed-loop (starter values; tune)
    public static final double kP = 0.0002;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // NOTE: velocityFF() is deprecated in REVLib in favor of feedForward config,
    // but it still works on most 2025 builds. If it errors, tell me and I’ll swap to feedForward.
    public static final double kFF = 0.00017;

    // Ready tolerance
    public static final double kRPMTolerance = 75.0;
  }

  private final SparkFlex shooterMotor =
      new SparkFlex(ShooterConstants.kShooterMotorCAN, MotorType.kBrushless);

  private final RelativeEncoder encoder = shooterMotor.getEncoder();
  private final SparkClosedLoopController pid = shooterMotor.getClosedLoopController();

  private double targetFlywheelRPM = 0.0;

  public ShooterSubsystem() {
    // REVLib 2025+ uses config objects + configure()
    SparkFlexConfig config = new SparkFlexConfig();

    config.inverted(ShooterConstants.kMotorInverted);

    // Closed-loop gains live in the config now
    config.closedLoop
        .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
        .outputRange(-1.0, 1.0);

    // If this line errors on your exact REVLib version, tell me and I'll switch to:
    // config.closedLoop.feedForward.<something...>
    config.closedLoop.velocityFF(ShooterConstants.kFF);

    // Apply config:
    // - Reset safe params so you're in a known state
    // - Persist so it survives brownouts/power cycles
    shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Sets flywheel RPM (NOT motor RPM). */
  public void setFlywheelRPM(double flywheelRPM) {
    flywheelRPM = MathUtil.clamp(flywheelRPM, ShooterConstants.kMinRPM, ShooterConstants.kMaxRPM);
    targetFlywheelRPM = flywheelRPM;

    // Convert flywheel RPM -> motor RPM based on gear ratio
    double motorRPM = flywheelRPM / ShooterConstants.kGearRatio;

    // REVLib 2025+: setSetpoint (setReference is deprecated)
    pid.setSetpoint(motorRPM, SparkBase.ControlType.kVelocity);
  }

  public void stop() {
    targetFlywheelRPM = 0.0;
    shooterMotor.stopMotor();
  }

  /** Returns flywheel RPM estimate from motor encoder RPM and gear ratio. */
  public double getFlywheelRPM() {
    double motorRPM = encoder.getVelocity(); // RPM
    return motorRPM * ShooterConstants.kGearRatio;
  }

  public double getTargetFlywheelRPM() {
    return targetFlywheelRPM;
  }

  public boolean atSetpoint() {
    return Math.abs(getFlywheelRPM() - targetFlywheelRPM) <= ShooterConstants.kRPMTolerance;
  }

  /** Convert desired exit speed (m/s) to flywheel RPM target. */
  public static double exitSpeedMpsToFlywheelRPM(double exitSpeedMps) {
    double wheelSurfaceSpeedMps = exitSpeedMps / ShooterConstants.kExitSpeedPerWheelSpeed;

    double omegaRadPerSec = wheelSurfaceSpeedMps / ShooterConstants.kFlywheelRadiusMeters;
    double wheelRPS = omegaRadPerSec / (2.0 * Math.PI);
    return wheelRPS * 60.0;
  }
}
