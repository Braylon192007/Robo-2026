package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HoodConstants;

// REVLib Servo Hub (CAN)
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;                 // <-- note: com.revrobotics.ResetMode (not ServoHub.ResetMode)
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig;

public class HoodSubsystem extends SubsystemBase {

  private final ServoHub servoHub = new ServoHub(HoodConstants.kServoHubCanId);

  private final ServoChannel hoodChannel =
      servoHub.getServoChannel(HoodConstants.kHoodChannelId);

  private double targetStrokeMm = HoodConstants.kMinStrokeMm;
  private int lastPulseUs = HoodConstants.kMinPulseUs;

  public HoodSubsystem() {
    // Build a channel config and apply it to the hub config (this replaces cfg.channel(...))
    ServoHubConfig cfg = new ServoHubConfig();

    ServoChannelConfig hoodCfg = new ServoChannelConfig(HoodConstants.kHoodChannelId)
        .pulseRange(HoodConstants.kMinPulseUs,
                    HoodConstants.kCenterPulseUs,
                    HoodConstants.kMaxPulseUs);

    // Apply per-channel config
    cfg.apply(HoodConstants.kHoodChannelId, hoodCfg);

    // Push config to device (persisting can be slow; do this once at startup)
    servoHub.configure(cfg, ResetMode.kResetSafeParameters);

    // Enable + power the channel
    hoodChannel.setPowered(true);
    hoodChannel.setEnabled(true);

    setStrokeMm(HoodConstants.kMinStrokeMm);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/TargetStrokeMm", targetStrokeMm);
    SmartDashboard.putNumber("Hood/LastPulseUs", lastPulseUs);
    SmartDashboard.putNumber("Hood/TargetAngleDeg", getTargetAngleDeg());
    SmartDashboard.putBoolean("Hood/Enabled", hoodChannel.isEnabled());
    SmartDashboard.putNumber("Hood/CurrentA", hoodChannel.getCurrent());
  }

  /** Set hood stroke in millimeters (mapped to pulse width). */
  public void setStrokeMm(double strokeMm) {
    strokeMm = MathUtil.clamp(strokeMm, HoodConstants.kMinStrokeMm, HoodConstants.kMaxStrokeMm);
    targetStrokeMm = strokeMm;

    // Map mm -> [0..1]
    double t = (strokeMm - HoodConstants.kMinStrokeMm) /
               (HoodConstants.kMaxStrokeMm - HoodConstants.kMinStrokeMm);
    t = MathUtil.clamp(t, 0.0, 1.0);

    // Map [0..1] -> pulse width (us)
    int pulseUs = (int) Math.round(
        HoodConstants.kMinPulseUs + t * (HoodConstants.kMaxPulseUs - HoodConstants.kMinPulseUs)
    );
    pulseUs = MathUtil.clamp(pulseUs, HoodConstants.kMinPulseUs, HoodConstants.kMaxPulseUs);

    lastPulseUs = pulseUs;

    REVLibError err = hoodChannel.setPulseWidth(pulseUs);
    SmartDashboard.putString("Hood/LastREVLibError", err.toString());
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

  public boolean atSetpoint() {
    return false;
  }

  /** Manual command in [0..1] where 0=retract, 1=extend */
  public void setPercent(double percent) {
    percent = MathUtil.clamp(percent, 0.0, 1.0);

    int pulseUs = (int) Math.round(
        HoodConstants.kMinPulseUs + percent * (HoodConstants.kMaxPulseUs - HoodConstants.kMinPulseUs)
    );
    pulseUs = MathUtil.clamp(pulseUs, HoodConstants.kMinPulseUs, HoodConstants.kMaxPulseUs);

    lastPulseUs = pulseUs;
    hoodChannel.setPulseWidth(pulseUs);

    targetStrokeMm = HoodConstants.kMinStrokeMm +
        percent * (HoodConstants.kMaxStrokeMm - HoodConstants.kMinStrokeMm);
  }

  public void stop() {
    hoodChannel.setPulseWidth(lastPulseUs);
  }
}
