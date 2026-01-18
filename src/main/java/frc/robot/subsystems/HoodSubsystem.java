package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

  public static class HoodConstants {
    public static final int kHoodCAN = 30; // <-- change

    // ✅ Phoenix 6 wants an enum, not boolean
    // Pick whichever makes "positive setpoint" move hood the direction you want.
    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    // If it goes the wrong way, swap to:
    // public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

    // Hood angle limits (deg)
    public static final double kMinAngleDeg = 50.0;
    public static final double kMaxAngleDeg = 60.0;

    // Conversion: hood degrees -> mechanism rotations
    public static final double kMechanismRotPerDeg = 1.0 / 360.0; // placeholder (fix later)

    // Sensor rotations / mechanism rotations (gearbox between motor and hood)
    public static final double kSensorToMechanismRatio = 1.0; // set if geared

    // Motion Magic constraints (mechanism rotations/sec and /sec^2)
    public static final double kCruiseVelocityRps = 2.0;
    public static final double kAccelerationRps2 = 6.0;

    // PID (starter)
    public static final double kP = 40.0;
    public static final double kI = 0.0;
    public static final double kD = 0.2;

    public static final double kAngleToleranceDeg = 1.0;
  }

  private final TalonFX hood = new TalonFX(HoodConstants.kHoodCAN);
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  private double targetAngleDeg = HoodConstants.kMinAngleDeg;

  public HoodSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Output
    cfg.MotorOutput.Inverted = HoodConstants.kInverted;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Ratio
    cfg.Feedback.SensorToMechanismRatio = HoodConstants.kSensorToMechanismRatio;

    // PID
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = HoodConstants.kP;
    slot0.kI = HoodConstants.kI;
    slot0.kD = HoodConstants.kD;
    cfg.Slot0 = slot0;

    // Motion Magic
    MotionMagicConfigs mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = HoodConstants.kCruiseVelocityRps;
    mm.MotionMagicAcceleration   = HoodConstants.kAccelerationRps2;
    cfg.MotionMagic = mm;

    hood.getConfigurator().apply(cfg);
  }

  /** Set hood angle in degrees. */
  public void setAngleDeg(double angleDeg) {
    angleDeg = MathUtil.clamp(angleDeg, HoodConstants.kMinAngleDeg, HoodConstants.kMaxAngleDeg);
    targetAngleDeg = angleDeg;

    double mechRot = angleDegToMechanismRot(angleDeg);
    hood.setControl(mmRequest.withPosition(mechRot));
  }

  public double getAngleDeg() {
    double mechRot = hood.getPosition().getValueAsDouble(); // mechanism rotations
    return mechanismRotToDeg(mechRot);
  }

  public double getTargetAngleDeg() {
    return targetAngleDeg;
  }

  public boolean atSetpoint() {
    return Math.abs(getAngleDeg() - targetAngleDeg) <= HoodConstants.kAngleToleranceDeg;
  }

  private static double angleDegToMechanismRot(double angleDeg) {
    return angleDeg * HoodConstants.kMechanismRotPerDeg;
  }

  private static double mechanismRotToDeg(double mechRot) {
    return mechRot / HoodConstants.kMechanismRotPerDeg;
  }
}
