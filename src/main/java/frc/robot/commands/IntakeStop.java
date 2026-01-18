package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStop extends Command {

  private final IntakeSubsystem intake;

  public IntakeStop(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return true; // instant
  }
}

