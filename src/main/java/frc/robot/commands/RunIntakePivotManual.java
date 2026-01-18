package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

public class RunIntakePivotManual extends Command {

  private final IntakePivotSubsystem pivot;
  private final double percent;

  /**
   * @param pivot IntakePivotSubsystem
   * @param percent Motor output (-1.0 to 1.0)
   */
  public RunIntakePivotManual(IntakePivotSubsystem pivot, double percent) {
    this.pivot = pivot;
    this.percent = percent;
    addRequirements(pivot);
  }

  @Override
  public void execute() {
    pivot.setPercent(percent);
  }

  @Override
  public void end(boolean interrupted) {
    pivot.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }
}
