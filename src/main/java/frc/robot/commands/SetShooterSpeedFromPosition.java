package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShooterMath;

public class SetShooterSpeedFromPosition extends Command {

  public static class Constants {
    public static final Translation2d kBlueScoreXY = new Translation2d(4.637, 4.072);
    public static final Translation2d kRedScoreXY  = new Translation2d(11.984, 4.072);

    public static final double kTargetHeightMeters = 1.8288; // 6 ft
    public static final double kShooterExitHeightMeters = 0.90; // measure this

    public static final double kMinExitSpeedMps = 3.0;
    public static final double kMaxExitSpeedMps = 25.0;

    public static final Translation2d kFallbackScoreXY = kBlueScoreXY;
  }

  private final ShooterSubsystem shooter;
  private final HoodSubsystem hood;
  private final Supplier<Translation2d> robotXYSupplier;

  public SetShooterSpeedFromPosition(
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      Supplier<Translation2d> robotXYSupplier) {

    this.shooter = shooter;
    this.hood = hood;
    this.robotXYSupplier = robotXYSupplier;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Translation2d robotXY = robotXYSupplier.get();
    Translation2d targetXY = getAllianceTarget();

    double x = robotXY.getDistance(targetXY);
    double y = Constants.kTargetHeightMeters - Constants.kShooterExitHeightMeters;

    double theta = Math.toRadians(hood.getAngleDeg());

    double v = ShooterMath.requiredExitSpeed(x, y, theta);
    if (!Double.isFinite(v)) {
      shooter.stop();
      return;
    }

    v = clamp(v, Constants.kMinExitSpeedMps, Constants.kMaxExitSpeedMps);
    shooter.setFlywheelRPM(ShooterSubsystem.exitSpeedMpsToFlywheelRPM(v));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static Translation2d getAllianceTarget() {
    Optional<Alliance> a = DriverStation.getAlliance();
    if (a.isPresent()) {
      return (a.get() == Alliance.Red)
          ? Constants.kRedScoreXY
          : Constants.kBlueScoreXY;
    }
    return Constants.kFallbackScoreXY;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
