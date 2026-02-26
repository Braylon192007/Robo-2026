package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Pickup extends Command {

  private final IntakeSubsystem intake;
  private final ConveyorSubsystem conveyor;

  public Pickup(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
    this.intake = intake;
    this.conveyor = conveyor;
    addRequirements(intake, conveyor);
  }

  @Override
  public void initialize() {
    intake.intake();
    conveyor.feed();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    conveyor.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }
}
