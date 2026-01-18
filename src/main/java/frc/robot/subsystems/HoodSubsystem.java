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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {

  

  private final TalonFX hood = new TalonFX(HoodConstants.kHoodCAN);
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  private double targetAngleDeg = HoodConstants.kMinAngleDeg;

  public HoodSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Output
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
  @Override
    public void periodic() {
      SmartDashboard.putNumber("Hood/MotorRot", hood.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Hood/AngleDeg", getAngleDeg());
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
    /** Manual open-loop percent output for tuning (-1 to 1). */
  public void setPercent(double percent) {
    hood.set(percent);
  }

  /** Stop hood motor. */
  public void stop() {
    hood.stopMotor();
  }

  /** Raw TalonFX position in rotations (useful for encoder testing). */
  public double getMotorRotations() {
    return hood.getPosition().getValueAsDouble();
  }

}
