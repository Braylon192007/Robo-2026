package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class SetHoodAngle extends Command {

  private final HoodSubsystem hood;
  private final DoubleSupplier angleDegSupplier;

  /**
   * @param hood Hood subsystem
   * @param angleDegSupplier Angle in degrees (constant or joystick/dashboard supplied)
   */
  public SetHoodAngle(HoodSubsystem hood, DoubleSupplier angleDegSupplier) {
    this.hood = hood;
    this.angleDegSupplier = angleDegSupplier;
    addRequirements(hood);
  }

  @Override
  public void execute() {
    hood.setAngleDeg(angleDegSupplier.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false; // run while scheduled
  }
}
