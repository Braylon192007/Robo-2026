package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX leftShooterMotor =
      new TalonFX(ShooterConstants.kLeftShooterMotorCAN);

  private final TalonFX rightShooterMotor =
      new TalonFX(ShooterConstants.kRightShooterMotorCAN);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final NeutralOut stopRequest = new NeutralOut();

  private double targetFlywheelRPM = 0.0;

  public ShooterSubsystem() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    leftConfig.Slot0 = new Slot0Configs()
        .withKP(ShooterConstants.kP)
        .withKI(ShooterConstants.kI)
        .withKD(ShooterConstants.kD)
        .withKV(ShooterConstants.kFF);

    rightConfig.Slot0 = new Slot0Configs()
        .withKP(ShooterConstants.kP)
        .withKI(ShooterConstants.kI)
        .withKD(ShooterConstants.kD)
        .withKV(ShooterConstants.kFF);

    leftConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(
            ShooterConstants.kLeftMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);

    rightConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(
            ShooterConstants.kRightMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);

    leftShooterMotor.getConfigurator().apply(leftConfig);
    rightShooterMotor.getConfigurator().apply(rightConfig);

    // Right follows left
    rightShooterMotor.setControl(
        new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        
  }

  /** Sets flywheel RPM (NOT motor RPM). */
  public void setFlywheelRPM(double flywheelRPM) {
    flywheelRPM = MathUtil.clamp(flywheelRPM, ShooterConstants.kMinRPM, ShooterConstants.kMaxRPM);
    targetFlywheelRPM = flywheelRPM;

    // Convert flywheel RPM -> motor RPS
    double motorRPM = flywheelRPM / ShooterConstants.kGearRatio;
    double motorRPS = motorRPM / 60.0;

    leftShooterMotor.setControl(velocityRequest.withVelocity(motorRPS));
  }
  public boolean atTargetRPM(double targetRPM, double toleranceRPM) {
    double leftMotorRPS = leftShooterMotor.getVelocity().getValueAsDouble();
    double rightMotorRPS = rightShooterMotor.getVelocity().getValueAsDouble();

    double leftFlywheelRPM = leftMotorRPS * 60.0 * ShooterConstants.kGearRatio;
    double rightFlywheelRPM = rightMotorRPS * 60.0 * ShooterConstants.kGearRatio;

    return Math.abs(targetRPM - leftFlywheelRPM) <= toleranceRPM
        && Math.abs(targetRPM - rightFlywheelRPM) <= toleranceRPM;
  }
  public void stop() {
    targetFlywheelRPM = 0.0;
    leftShooterMotor.setControl(stopRequest);
    rightShooterMotor.setControl(stopRequest);
  }

  /** Returns flywheel RPM estimate from motor encoder velocity and gear ratio. */
  public double getFlywheelRPM() {
    double leftMotorRPS = leftShooterMotor.getVelocity().getValueAsDouble();
    double rightMotorRPS = rightShooterMotor.getVelocity().getValueAsDouble();

    double leftFlywheelRPM = leftMotorRPS * 60.0 * ShooterConstants.kGearRatio;
    double rightFlywheelRPM = rightMotorRPS * 60.0 * ShooterConstants.kGearRatio;

    return (leftFlywheelRPM + rightFlywheelRPM) / 2.0;
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