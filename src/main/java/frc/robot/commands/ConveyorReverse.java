package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorReverse extends Command {

  private final ConveyorSubsystem conveyor;

  public ConveyorReverse(ConveyorSubsystem conveyor) {
    this.conveyor = conveyor;
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    conveyor.reverse();
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }
}
