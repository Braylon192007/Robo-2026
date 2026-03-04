package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PivotConstants;

public class IntakePivotSubsystem extends SubsystemBase {

  private final SparkMax pivot =
      new SparkMax(PivotConstants.kPivotMotorCAN, MotorType.kBrushless);

  private final RelativeEncoder encoder = pivot.getEncoder();

  // Simple open-loop "drive down" settings
  private static final double kDownPower = 0.35;          // tune this
  private static final double kUpPower = -0.35;           // tune if you want up later
  private static final double kStopWindowDeg = 1.5;       // stop when within this of target

  private double targetDeg = PivotConstants.kUpDeg;
  private boolean movingToTarget = false;

  public IntakePivotSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(PivotConstants.kMotorInverted);
    config.idleMode(IdleMode.kBrake); // brake helps "bumper holds it"
    config.smartCurrentLimit(PivotConstants.kCurrentLimitA);

    pivot.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Optional: assume robot boots with intake UP, so set encoder to "up"
    zeroUpHere();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakePivot/AngleDeg", getAngleDeg());
    SmartDashboard.putNumber("IntakePivot/TargetDeg", targetDeg);
    SmartDashboard.putBoolean("IntakePivot/MovingToTarget", movingToTarget);

    if (!movingToTarget) {
      return;
    }

    double currentDeg = getAngleDeg();
    double errorDeg = targetDeg - currentDeg;

    // Stop when close enough
    if (Math.abs(errorDeg) <= kStopWindowDeg) {
      pivot.stopMotor();
      movingToTarget = false;
      return;
    }

    // Simple "bang-bang" direction control
    // NOTE: If your sign is backwards, swap kDownPower/kUpPower or flip the comparisons.
    if (errorDeg > 0.0) {
      // need to increase angle -> go "down" (usually)
      pivot.set(-kDownPower);
    } else {
      // need to decrease angle -> go "up" (usually)
      pivot.set(-kUpPower);
    }
  }

  /** Zero encoder when intake is physically UP */
  public void zeroUpHere() {
    encoder.setPosition(0.0);
    targetDeg = PivotConstants.kUpDeg;
    movingToTarget = false;
    pivot.stopMotor();
  }

  /** Current pivot angle in degrees */
  public double getAngleDeg() {
    return encoder.getPosition() / PivotConstants.kMotorRotPerDeg;
  }

  /** Move to a target angle using simple open-loop power, then stop */
  public void setAngleDegSimple(double angleDeg) {
    targetDeg = MathUtil.clamp(angleDeg, PivotConstants.kMinDeg, PivotConstants.kMaxDeg);
    movingToTarget = true;
  }

  /** Call this to go to the down position (simple, no holding) */
  public void intakeDown() {
    setAngleDegSimple(PivotConstants.kDownDeg);
  }

  /** Optional: simple up */
  public void intakeUp() {
    setAngleDegSimple(PivotConstants.kUpDeg);
  }

  /** Manual control (cancels any move-to-target) */
  public void setPercent(double percent) {
    movingToTarget = false;
    pivot.set(percent);
  }

  public void stop() {
    movingToTarget = false;
    pivot.stopMotor();
  }

  public boolean atTarget() {
    return Math.abs(getAngleDeg() - targetDeg) <= kStopWindowDeg;
  }
}