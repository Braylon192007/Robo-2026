package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {

  // WCP-0412 is an RC-servo style actuator: command position via PWM
  private final Servo hoodServo = new Servo(HoodConstants.kHoodPwmChannel);

  private double targetStrokeMm = HoodConstants.kMinStrokeMm;

  public HoodSubsystem() {
    // WCP docs: 1.0ms = fully retract, 2.0ms = fully extend. :contentReference[oaicite:2]{index=2}
    // Configure servo pulse bounds to match the actuator.
    // (max, deadbandMax, center, deadbandMin, min) in microseconds
    hoodServo.setBoundsMicroseconds(2000, 0, 1500, 0, 1000);

    // Optional: start at a safe known angle
    setStrokeMm(HoodConstants.kMinStrokeMm);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/TargetStrokeMm", targetStrokeMm);
    SmartDashboard.putNumber("Hood/CommandedPWM(0-1)", hoodServo.get());
    // NOTE: This is NOT actual position feedback, just what we're commanding.
  }

  /** Set hood angle in degrees (mapped across actuator stroke). */
  public void setStrokeMm(double strokeMm) {
  strokeMm = MathUtil.clamp(strokeMm, HoodConstants.kMinStrokeMm, HoodConstants.kMaxStrokeMm);
  targetStrokeMm = strokeMm;

  // Map mm -> servo position [0..1]
  double pos = (strokeMm - HoodConstants.kMinStrokeMm) /
               (HoodConstants.kMaxStrokeMm - HoodConstants.kMinStrokeMm);

  hoodServo.set(MathUtil.clamp(pos, 0.0, 1.0));
}

  /** Returns the last requested target angle (not measured). */
  public double getTargetStrokeMm() {
    return targetStrokeMm;
  }

  /** With no sensor, "at setpoint" can only be approximate/time-based. */
  public boolean atSetpoint() {
    // If you add a potentiometer/encoder later, swap this to real feedback.
    return false;
  }

  /** Manual command in [0..1] where 0=retract, 1=extend */
  public void setPercent(double percent) {
    hoodServo.set(MathUtil.clamp(percent, 0.0, 1.0));
    // keep targetStrokeMm in sync (optional)
    targetStrokeMm = HoodConstants.kMinStrokeMm +
        hoodServo.get() * (HoodConstants.kMaxStrokeMm - HoodConstants.kMinStrokeMm);
  }

  /** Stop sending movement commands (holds last position on most servo actuators). */
  public void stop() {
    // You can either leave it alone (it will keep holding), or set to the same value.
    hoodServo.set(hoodServo.get());
  }
}
