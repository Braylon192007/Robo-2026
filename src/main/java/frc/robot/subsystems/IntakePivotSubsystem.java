package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.PivotConstants;

public class IntakePivotSubsystem extends SubsystemBase {

  

  private final TalonFX pivot = new TalonFX(PivotConstants.kPivotMotorCAN);
  private final MotionMagicVoltage mm = new MotionMagicVoltage(0);

  private double targetDeg = PivotConstants.kUpDeg;

  public IntakePivotSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted = PivotConstants.kMotorInverted;
    cfg.MotorOutput.NeutralMode = PivotConstants.kNeutralMode;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = PivotConstants.kP;
    slot0.kI = PivotConstants.kI;
    slot0.kD = PivotConstants.kD;
    cfg.Slot0 = slot0;

    MotionMagicConfigs mmCfg = new MotionMagicConfigs();
    mmCfg.MotionMagicCruiseVelocity = PivotConstants.kCruiseRotPerSec;
    mmCfg.MotionMagicAcceleration   = PivotConstants.kAccelRotPerSec2;
    cfg.MotionMagic = mmCfg;

    pivot.getConfigurator().apply(cfg);

    // Start with current motor position treated as "up=0" until you zero properly
    zeroUpHere();
  }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakePivot/MotorRot", pivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakePivot/AngleDeg", getAngleDeg());
    }
  /**
   * Call this when the intake is physically all the way UP (hard stop),
   * so Up becomes 0.
   */
  public void zeroUpHere() {
    pivot.setPosition(0.0); // motor sensor position becomes 0 rotations
    targetDeg = PivotConstants.kUpDeg;
  }

  /** Pivot angle in degrees (based on motor rotations and kMotorRotPerDeg). */
  public double getAngleDeg() {
    double motorRot = pivot.getPosition().getValueAsDouble(); // rotations
    return motorRot / PivotConstants.kMotorRotPerDeg;
  }

  /** Command pivot angle in degrees (Up = 0). */
  public void setAngleDeg(double angleDeg) {
    angleDeg = MathUtil.clamp(angleDeg, PivotConstants.kMinDeg, PivotConstants.kMaxDeg);
    targetDeg = angleDeg;

    double motorRotTarget = angleDeg * PivotConstants.kMotorRotPerDeg;
    pivot.setControl(mm.withPosition(motorRotTarget));
  }

  /** Manual percent output (use small values for testing). */
  public void setPercent(double percent) {
    pivot.set(percent);
  }

  public void stop() {
    pivot.stopMotor();
  }

  public double getTargetDeg() {
    return targetDeg;
  }

  public boolean atSetpoint() {
    return Math.abs(getAngleDeg() - targetDeg) <= PivotConstants.kToleranceDeg;
  }
}
