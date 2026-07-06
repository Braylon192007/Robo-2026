package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetPoseBasedOnLimelight extends CommandBase {
	private final Supplier<Boolean> hasTargetSupplier;
	private final Supplier<Pose2d> limelightPoseSupplier;
	private final Consumer<Pose2d> resetPoseConsumer;

	public ResetPoseBasedOnLimelight(
			Supplier<Boolean> hasTargetSupplier,
			Supplier<Pose2d> limelightPoseSupplier,
			Consumer<Pose2d> resetPoseConsumer) {
		this.hasTargetSupplier = hasTargetSupplier;
		this.limelightPoseSupplier = limelightPoseSupplier;
		this.resetPoseConsumer = resetPoseConsumer;
	}

	@Override
	public void initialize() {
		if (hasTargetSupplier.get()) {
			resetPoseConsumer.accept(limelightPoseSupplier.get());
		}
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
