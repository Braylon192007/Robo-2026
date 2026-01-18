package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
public class ConveyorSubsystem extends SubsystemBase {

  

  private final SparkFlex conveyorMotor =
      new SparkFlex(ConveyorConstants.kConveyorMotorCAN, MotorType.kBrushless);

  public ConveyorSubsystem() {
    SparkFlexConfig cfg = new SparkFlexConfig();

    cfg.smartCurrentLimit(ConveyorConstants.kCurrentLimit);
    cfg.openLoopRampRate(ConveyorConstants.kOpenLoopRampRate);

    conveyorMotor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  /** Feed balls toward the indexer. */
  public void feed() {
    conveyorMotor.set(ConveyorConstants.kFeedSpeed);
  }

  /** Reverse conveyor to clear jams or spit balls out. */
  public void reverse() {
    conveyorMotor.set(ConveyorConstants.kReverseSpeed);
  }

  /** Stop conveyor. */
  public void stop() {
    conveyorMotor.stopMotor();
  }

  /** Raw percent output (for tuning/debug). */
  public void set(double percent) {
    conveyorMotor.set(percent);
  }
}
