package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HoodConstants;

import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig;

public class HoodSubsystem extends SubsystemBase {

  private final ServoHub servoHub = new ServoHub(HoodConstants.kServoHubCanId);

  private final ServoChannel left =
      servoHub.getServoChannel(HoodConstants.kHoodChannelIdLeft);

  private final ServoChannel right =
      servoHub.getServoChannel(HoodConstants.kHoodChannelIdRight);

  private double targetStrokeMm = HoodConstants.kMinStrokeMm;

  private int lastPulseLeftUs  = HoodConstants.kMinPulseUs;
  private int lastPulseRightUs = HoodConstants.kMinPulseUs;

  public HoodSubsystem() {

    // Configure BOTH channels (pulse range)
    ServoHubConfig hubCfg = new ServoHubConfig();

    ServoChannelConfig leftCfg = new ServoChannelConfig(HoodConstants.kHoodChannelIdLeft)
        .pulseRange(HoodConstants.kMinPulseUs, HoodConstants.kCenterPulseUs, HoodConstants.kMaxPulseUs);

    ServoChannelConfig rightCfg = new ServoChannelConfig(HoodConstants.kHoodChannelIdRight)
        .pulseRange(HoodConstants.kMinPulseUs, HoodConstants.kCenterPulseUs, HoodConstants.kMaxPulseUs);

    // Apply per-channel configs to the hub config
    hubCfg.apply(HoodConstants.kHoodChannelIdLeft, leftCfg);
    hubCfg.apply(HoodConstants.kHoodChannelIdRight, rightCfg);

    // Push config to device
    servoHub.configure(hubCfg, ResetMode.kResetSafeParameters);

    // Enable + power both channels
    enableChannel(left, true);
    enableChannel(right, true);

    // Start at min stroke
    setStrokeMm(HoodConstants.kMinStrokeMm);
  }

  private static void enableChannel(ServoChannel ch, boolean enabled) {
    ch.setPowered(enabled);
    ch.setEnabled(enabled);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/TargetStrokeMm", targetStrokeMm);
    SmartDashboard.putNumber("Hood/TargetAngleDeg", getTargetAngleDeg());

    SmartDashboard.putNumber("Hood/LeftPulseUs", lastPulseLeftUs);
    SmartDashboard.putNumber("Hood/RightPulseUs", lastPulseRightUs);

    SmartDashboard.putBoolean("Hood/LeftEnabled", left.isEnabled());
    SmartDashboard.putBoolean("Hood/RightEnabled", right.isEnabled());

    SmartDashboard.putNumber("Hood/LeftCurrentA", left.getCurrent());
    SmartDashboard.putNumber("Hood/RightCurrentA", right.getCurrent());
  }

  /** Set hood stroke in millimeters (mapped to pulse width). */
  public void setStrokeMm(double strokeMm) {
    strokeMm = MathUtil.clamp(strokeMm, HoodConstants.kMinStrokeMm, HoodConstants.kMaxStrokeMm);
    targetStrokeMm = strokeMm;

    int pulse = strokeMmToPulseUs(strokeMm);

    // If mirrored mechanically, flip one side around center.
    int leftPulse  = pulse;
    int rightPulse = HoodConstants.kInvertRightPulse ? mirrorPulseAboutCenter(pulse) : pulse;

    lastPulseLeftUs = leftPulse;
    lastPulseRightUs = rightPulse;

    REVLibError errL = left.setPulseWidth(leftPulse);
    REVLibError errR = right.setPulseWidth(rightPulse);

    SmartDashboard.putString("Hood/LastREVErrLeft", errL.toString());
    SmartDashboard.putString("Hood/LastREVErrRight", errR.toString());
  }

  /** Manual command in [0..1] where 0=retract, 1=extend */
  public void setPercent(double percent) {
    percent = MathUtil.clamp(percent, 0.0, 1.0);
    double stroke = HoodConstants.kMinStrokeMm +
        percent * (HoodConstants.kMaxStrokeMm - HoodConstants.kMinStrokeMm);
    setStrokeMm(stroke);
  }

  /** Optional: command by angle if your stroke->angle is linear. */
  public void setAngleDeg(double angleDeg) {
    angleDeg = MathUtil.clamp(angleDeg, HoodConstants.kMinAngleDeg, HoodConstants.kMaxAngleDeg);

    double t = (angleDeg - HoodConstants.kMinAngleDeg) /
               (HoodConstants.kMaxAngleDeg - HoodConstants.kMinAngleDeg);
    t = MathUtil.clamp(t, 0.0, 1.0);

    double stroke = HoodConstants.kMinStrokeMm +
        t * (HoodConstants.kMaxStrokeMm - HoodConstants.kMinStrokeMm);

    setStrokeMm(stroke);
  }

  public double getTargetStrokeMm() {
    return targetStrokeMm;
  }

  public double getTargetAngleDeg() {
    double t = (targetStrokeMm - HoodConstants.kMinStrokeMm) /
               (HoodConstants.kMaxStrokeMm - HoodConstants.kMinStrokeMm);
    t = MathUtil.clamp(t, 0.0, 1.0);

    return HoodConstants.kMinAngleDeg +
        t * (HoodConstants.kMaxAngleDeg - HoodConstants.kMinAngleDeg);
  }

  /** No real feedback, so this can't be true position-based. */
  public boolean atSetpoint() {
    return false;
  }

  /** Keep last command (servos hold their last pulse). */
  public void stop() {
    left.setPulseWidth(lastPulseLeftUs);
    right.setPulseWidth(lastPulseRightUs);
  }

  // --- helpers ---

  private static int strokeMmToPulseUs(double strokeMm) {
    double t = (strokeMm - HoodConstants.kMinStrokeMm) /
               (HoodConstants.kMaxStrokeMm - HoodConstants.kMinStrokeMm);
    t = MathUtil.clamp(t, 0.0, 1.0);

    int pulseUs = (int) Math.round(
        HoodConstants.kMinPulseUs + t * (HoodConstants.kMaxPulseUs - HoodConstants.kMinPulseUs)
    );

    return MathUtil.clamp(pulseUs, HoodConstants.kMinPulseUs, HoodConstants.kMaxPulseUs);
  }

  private static int mirrorPulseAboutCenter(int pulseUs) {
    int center = HoodConstants.kCenterPulseUs;
    int mirrored = center - (pulseUs - center);
    return MathUtil.clamp(mirrored, HoodConstants.kMinPulseUs, HoodConstants.kMaxPulseUs);
  }
}
