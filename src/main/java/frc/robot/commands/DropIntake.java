package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.Constants.PivotConstants;

public class DropIntake extends Command {

  private final IntakePivotSubsystem pivot;

  public DropIntake(IntakePivotSubsystem pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivot.setAngleDeg(PivotConstants.kDownDeg); // 75°
  }

  @Override
  public boolean isFinished() {
    return pivot.atSetpoint();
  }
}
