package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {

  private final TalonFX hoodMotor =
      new TalonFX(HoodConstants.kHoodMotorCAN);

  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final NeutralOut stopRequest = new NeutralOut();

  private double targetAngleDeg = HoodConstants.kMinAngleDeg;

  public HoodSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        HoodConstants.kMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.Slot0 = new Slot0Configs()
        .withKP(HoodConstants.kP)
        .withKI(HoodConstants.kI)
        .withKD(HoodConstants.kD)
        .withKG(HoodConstants.kG)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    //config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
    //    .withForwardSoftLimitEnable(true)
    //    .withForwardSoftLimitThreshold(angleDegToMotorRot(HoodConstants.kMaxAngleDeg))
    //    .withReverseSoftLimitEnable(true)
     //   .withReverseSoftLimitThreshold(angleDegToMotorRot(HoodConstants.kMinAngleDeg));

    hoodMotor.getConfigurator().apply(config);

    // If your hood is not physically sitting at min angle at boot,
    // change this to whatever real starting angle it is.
    hoodMotor.setPosition(angleDegToMotorRot(HoodConstants.kStartingAngleDeg));
    targetAngleDeg = HoodConstants.kStartingAngleDeg;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/TargetAngleDeg", targetAngleDeg);
    SmartDashboard.putNumber("Hood/CurrentAngleDeg", getAngleDeg());
    SmartDashboard.putNumber("Hood/MotorRotations", hoodMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood/VelocityRPS", hoodMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Hood/AtSetpoint", atSetpoint());
  }

  public void setAngleDeg(double angleDeg) {
    angleDeg = MathUtil.clamp(angleDeg, HoodConstants.kMinAngleDeg, HoodConstants.kMaxAngleDeg);
    targetAngleDeg = angleDeg;

    double motorRotations = angleDegToMotorRot(angleDeg);
    hoodMotor.setControl(positionRequest.withPosition(motorRotations));
  }

  public void setPercent(double percent) {
    percent = MathUtil.clamp(percent, 0.0, 1.0);

    double angleDeg = HoodConstants.kMinAngleDeg
        + percent * (HoodConstants.kMaxAngleDeg - HoodConstants.kMinAngleDeg);

    setAngleDeg(angleDeg);
  }

  public double getAngleDeg() {
    return motorRotToAngleDeg(hoodMotor.getPosition().getValueAsDouble());
  }

  public double getTargetAngleDeg() {
    return targetAngleDeg;
  }

  public boolean atSetpoint() {
    return Math.abs(getAngleDeg() - targetAngleDeg) <= HoodConstants.kAngleToleranceDeg;
  }

  public void stop() {
    hoodMotor.setControl(stopRequest);
  }
  public void setSpeed(double speed) {
    hoodMotor.set(speed);
  }

  private static double angleDegToMotorRot(double angleDeg) {
    double hoodRotations = (angleDeg - HoodConstants.kMinAngleDeg) / 360.0;
    return hoodRotations * HoodConstants.kGearRatio;
  }

  private static double motorRotToAngleDeg(double motorRotations) {
    double hoodRotations = motorRotations / HoodConstants.kGearRatio;
    return HoodConstants.kMinAngleDeg + hoodRotations * 360.0;
  }
}