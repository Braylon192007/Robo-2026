package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class RunHoodManual extends Command {

  private final HoodSubsystem hood;
  private final double percent;

  /**
   * @param hood HoodSubsystem
   * @param percent Motor output (-1.0 to 1.0)
   */
  public RunHoodManual(HoodSubsystem hood, double percent) {
    this.hood = hood;
    this.percent = percent;
    addRequirements(hood);
  }

  @Override
  public void execute() {
    hood.setPercent(percent);
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }
}
