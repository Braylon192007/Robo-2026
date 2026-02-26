package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class FeedBall extends Command {

  private final IndexerSubsystem indexer;
  private final ConveyorSubsystem conveyor;

  public FeedBall(IndexerSubsystem indexer, ConveyorSubsystem conveyor) {
    this.indexer = indexer;
    this.conveyor = conveyor;
    addRequirements(indexer, conveyor);
  }

  @Override
  public void initialize() {
    indexer.feed();
    conveyor.feed();
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    conveyor.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }
}
