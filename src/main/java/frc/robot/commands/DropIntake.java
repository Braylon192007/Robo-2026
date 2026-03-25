package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.IntakePivotSubsystem;

public class DropIntake extends WrapperCommand {
  public DropIntake(IntakePivotSubsystem m_intakePivotSubsystem) {
    super(
      Commands.sequence(
        Commands.runOnce(() -> m_intakePivotSubsystem.setPercent(-0.4), m_intakePivotSubsystem),
        Commands.waitSeconds(0.7),
        Commands.runOnce(() -> m_intakePivotSubsystem.stop(), m_intakePivotSubsystem)
    ));
  }
}
