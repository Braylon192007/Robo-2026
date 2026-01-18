package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
public class IndexerSubsystem extends SubsystemBase {

  

  private final SparkFlex indexerMotor =
      new SparkFlex(IndexerConstants.kIndexerMotorCAN, MotorType.kBrushless);

  public IndexerSubsystem() {
    SparkFlexConfig cfg = new SparkFlexConfig();

    cfg.smartCurrentLimit(IndexerConstants.kCurrentLimit);
    cfg.openLoopRampRate(IndexerConstants.kOpenLoopRampRate);

    indexerMotor.configure(cfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }
  

  /** Feed the ball upward into the shooter. */
  public void feed() {
    indexerMotor.set(IndexerConstants.kFeedSpeed);
  }

  /** Reverse the indexer to clear jams / pull ball back. */
  public void reverse() {
    indexerMotor.set(IndexerConstants.kReverseSpeed);
  }

  /** Stop indexer. */
  public void stop() {
    indexerMotor.stopMotor();
  }

  /** Raw percent output (tuning/debug). */
  public void set(double percent) {
    indexerMotor.set(percent);
  }
}
