package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
public class IntakeOut extends Command {

  private final IntakeSubsystem intake;
  private final ConveyorSubsystem conveyor;

  public IntakeOut(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
    this.intake = intake;
    this.conveyor = conveyor;
    addRequirements(intake, conveyor);
  }

  @Override
  public void initialize() {
    intake.outtake();
    conveyor.reverse();
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

