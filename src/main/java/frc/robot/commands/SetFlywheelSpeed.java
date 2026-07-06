package frc.robot.commands;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SetFlywheelSpeed extends CommandBase {
	private final DoubleConsumer setSpeed;
	private final Runnable stop;
	private final double targetSpeed;

	public SetFlywheelSpeed(Subsystem requirement, DoubleConsumer setSpeed, Runnable stop, double targetSpeed) {
		this.setSpeed = setSpeed;
		this.stop = stop;
		this.targetSpeed = targetSpeed;
		addRequirements(requirement);
	}

	@Override
	public void execute() {
		setSpeed.accept(targetSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		stop.run();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
