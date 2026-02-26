package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor =
      new SparkMax(ClimberConstants.kClimberMotorCAN, MotorType.kBrushless);
    
    private final RelativeEncoder encoder = climberMotor.getEncoder();
    
    private double targetPos = 0.0;
    @SuppressWarnings("removal")
    public ClimberSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(ClimberConstants.kInverted);
    cfg.smartCurrentLimit(ClimberConstants.kCurrentLimit);
    cfg.openLoopRampRate(ClimberConstants.kOpenLoopRampRate);
    
    climberMotor.configure(cfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void climberUp() {
        climberMotor.set(ClimberConstants.kClimbSpeed);
    }
    public void climberDown() {
        climberMotor.set(-ClimberConstants.kClimbSpeed);
    }
    public void stop() {
        climberMotor.stopMotor();
    }
    public void climberTopPosition() {
        targetPos = 90.0; // <-- change
        if (encoder.getPosition() < targetPos) {
            climberUp();
        } else if (encoder.getPosition() > targetPos) {
            climberDown();
        } else {
            stop();
        }
    }
    public void climberBottomPosition() {
        targetPos = 5.0; // <-- change
        if (encoder.getPosition() > targetPos) {
            climberDown();
        } else if (encoder.getPosition() < targetPos) {
            climberUp();
        } else {
            stop();
        }
    }
    public double getPosition() {
        return encoder.getPosition();
    }
}