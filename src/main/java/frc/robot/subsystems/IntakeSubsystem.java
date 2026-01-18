package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
public class IntakeSubsystem extends SubsystemBase {

  

  private final SparkMax intakeMotor =
      new SparkMax(IntakeConstants.kIntakeMotorCAN, MotorType.kBrushless);

  @SuppressWarnings("removal")
  public IntakeSubsystem() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(IntakeConstants.kInverted);
    cfg.smartCurrentLimit(IntakeConstants.kCurrentLimit);
    
    intakeMotor.configure(cfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  public void intake() {
    intakeMotor.set(IntakeConstants.kIntakeSpeed);
  }

  public void outtake() {
    intakeMotor.set(IntakeConstants.kOuttakeSpeed);
  }

  /** Stop intake motor. */
  public void stop() {
    intakeMotor.stopMotor();
  }

  /** Optional: set raw percent output (for tuning/debug). */
  public void set(double percent) {
    intakeMotor.set(percent);
  }
}
