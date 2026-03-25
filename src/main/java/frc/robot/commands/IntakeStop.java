package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStop extends WrapperCommand {
  public IntakeStop(IntakeSubsystem m_intakeSubsystem, ConveyorSubsystem m_conveyorSubsystem) {
    super(
      Commands.parallel(
        Commands.runOnce(() -> m_intakeSubsystem.stop(), m_intakeSubsystem),
        Commands.runOnce(() -> m_conveyorSubsystem.stop(), m_conveyorSubsystem)
    ));
  }
}
