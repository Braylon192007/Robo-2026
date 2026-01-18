package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.Constants.PivotConstants;
public class RetractIntake extends Command {

  private final IntakePivotSubsystem pivot;

  public RetractIntake(IntakePivotSubsystem pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivot.setAngleDeg(PivotConstants.kUpDeg); // 0°
  }

  @Override
  public boolean isFinished() {
    return pivot.atSetpoint();
  }
}
