package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class SetHoodAngle extends Command {

  private final HoodSubsystem hood;
  private final DoubleSupplier strokeMmSupplier;

  /**
   * @param hood Hood subsystem
   * @param strokeMmSupplier Stroke in millimeters (constant or joystick/dashboard supplied)
   */
  public SetHoodAngle(HoodSubsystem hood, DoubleSupplier strokeMmSupplier) {
    this.hood = hood;
    this.strokeMmSupplier = strokeMmSupplier;
    addRequirements(hood);
  }

  @Override
  public void execute() {
    hood.setStrokeMm(strokeMmSupplier.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false; // run while scheduled
  }
}
