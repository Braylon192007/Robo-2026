package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

public class DropIntake extends Command {

  private final IntakePivotSubsystem pivot;

  public DropIntake(IntakePivotSubsystem pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    // Start the movement ONCE
    pivot.intakeUp();
  }

  @Override
  public boolean isFinished() {
    // Finish when we reach the target
    return pivot.atTarget();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop motor when done
    pivot.stop();
  }
}