// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAll extends ParallelCommandGroup {
  /** Creates a new StopAll. */
  public StopAll(ConveyorSubsystem m_conveyorSubsystem, IndexerSubsystem m_indexerSubsystem, ShooterSubsystem m_shooterSubsystem, IntakeSubsystem m_intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(            
      Commands.runOnce(() -> m_conveyorSubsystem.stop(), m_conveyorSubsystem),
      Commands.runOnce(() -> m_indexerSubsystem.stop(), m_indexerSubsystem),
      Commands.runOnce(() -> m_shooterSubsystem.stop(), m_shooterSubsystem),
      Commands.runOnce(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
    );
  }
}
