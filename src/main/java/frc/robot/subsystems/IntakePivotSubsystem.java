package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PivotConstants;

public class IntakePivotSubsystem extends SubsystemBase {

  private final SparkMax pivot =
      new SparkMax(PivotConstants.kPivotMotorCAN, MotorType.kBrushless);

  private final SparkClosedLoopController controller =
      pivot.getClosedLoopController();

  private final RelativeEncoder encoder = pivot.getEncoder();

  private double targetDeg = PivotConstants.kUpDeg;
  private boolean holding = true;

  public IntakePivotSubsystem() {

    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(PivotConstants.kMotorInverted);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(PivotConstants.kCurrentLimitA);

    // PID
    config.closedLoop
        .pid(
            PivotConstants.kP,
            PivotConstants.kI,
            PivotConstants.kD)
        .outputRange(-1.0, 1.0);

    // MAXMotion settings
    config.closedLoop.maxMotion
        .cruiseVelocity(PivotConstants.kCruiseRotPerSec)
        .maxAcceleration(PivotConstants.kAccelRotPerSec2)
        .allowedProfileError(PivotConstants.kAllowedProfileErrorRot);

    pivot.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero on boot (assumes robot powers on with intake UP)
    zeroUpHere();
    holdAt(targetDeg);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakePivot/AngleDeg", getAngleDeg());
    SmartDashboard.putNumber("IntakePivot/TargetDeg", targetDeg);
    SmartDashboard.putBoolean("IntakePivot/Holding", holding);

    // Re-assert hold every loop so bumps are corrected immediately
    if (holding) {
      double motorRotTarget =
          targetDeg * PivotConstants.kMotorRotPerDeg;

      controller.setSetpoint(
          motorRotTarget,
          SparkBase.ControlType.kMAXMotionPositionControl);
    }
  }

  /** Zero encoder when intake is physically UP */
  public void zeroUpHere() {
    encoder.setPosition(0.0);
    targetDeg = PivotConstants.kUpDeg;
  }

  /** Current pivot angle in degrees */
  public double getAngleDeg() {
    return encoder.getPosition() / PivotConstants.kMotorRotPerDeg;
  }

  /** Move to angle and HOLD it */
  public void setAngleDeg(double angleDeg) {
    angleDeg = MathUtil.clamp(
        angleDeg,
        PivotConstants.kMinDeg,
        PivotConstants.kMaxDeg);

    targetDeg = angleDeg;
    holdAt(angleDeg);
  }
  public void IntakeDown() {
    setAngleDeg(PivotConstants.kDownDeg);
  }
  public void IntakeUp() {
    setAngleDeg(PivotConstants.kUpDeg);
  }

  /** Hold wherever the pivot currently is */
  public void holdCurrent() {
    targetDeg = getAngleDeg();
    holdAt(targetDeg);
  }

  private void holdAt(double angleDeg) {
    holding = true;

    double motorRotTarget =
        angleDeg * PivotConstants.kMotorRotPerDeg;

    controller.setSetpoint(
        motorRotTarget,
        SparkBase.ControlType.kMAXMotionPositionControl);
  }

  /** Manual control (disables hold) */
  public void setPercent(double percent) {
    holding = false;
    pivot.set(percent);
  }

  public void stop() {
    holding = false;
    pivot.stopMotor();
  }

  public boolean atSetpoint() {
    return Math.abs(getAngleDeg() - targetDeg)
        <= PivotConstants.kToleranceDeg;
  }
}
