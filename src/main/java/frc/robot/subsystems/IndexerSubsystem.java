package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax topIndexerMotor =
      new SparkMax(IndexerConstants.kIndexerMotorCANTop, MotorType.kBrushless);

  private final SparkMax bottomIndexerMotor =
      new SparkMax(IndexerConstants.kIndexerMotorCANBottom, MotorType.kBrushless);

  public IndexerSubsystem() {
    SparkMaxConfig topCfg = new SparkMaxConfig();
    SparkMaxConfig bottomCfg = new SparkMaxConfig();

    topCfg
        .smartCurrentLimit(IndexerConstants.kCurrentLimit)
        .openLoopRampRate(IndexerConstants.kOpenLoopRampRate)
        .idleMode(IdleMode.kCoast)
        .inverted(IndexerConstants.kTopInverted);

    bottomCfg
        .smartCurrentLimit(IndexerConstants.kCurrentLimit)
        .openLoopRampRate(IndexerConstants.kOpenLoopRampRate)
        .idleMode(IdleMode.kCoast)
        .inverted(IndexerConstants.kBottomInverted);

    topIndexerMotor.configure(
        topCfg,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    bottomIndexerMotor.configure(
        bottomCfg,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /** Feed the note into the shooter. */
  public void feed() {
    topIndexerMotor.set(IndexerConstants.kFeedSpeed);
    bottomIndexerMotor.set(IndexerConstants.kFeedSpeed);
  }

  /** Reverse both indexers to clear jams / back note out. */
  public void reverse() {
    topIndexerMotor.set(IndexerConstants.kReverseSpeed);
    bottomIndexerMotor.set(IndexerConstants.kReverseSpeed);
    bottomIndexerMotor.set(IndexerConstants.kReverseSpeed);
  }

  /** Stop both indexers. */
  public void stop() {
    topIndexerMotor.stopMotor();
    bottomIndexerMotor.stopMotor();
  }

  /** Raw percent output to both motors. */
  public void set(double percent) {
    topIndexerMotor.set(percent);
    bottomIndexerMotor.set(percent);
  }

  /** Raw separate control if you want tuning/debugging. */
  public void set(double topPercent, double bottomPercent) {
    topIndexerMotor.set(topPercent);
    bottomIndexerMotor.set(bottomPercent);
  }
}