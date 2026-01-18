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

public class AimHoodAndShooter extends Command {

  public static class Constants {
    public static final Translation2d kBlueScoreXY = new Translation2d(4.637, 4.072);
    public static final Translation2d kRedScoreXY  = new Translation2d(11.984, 4.072);

    public static final double kTargetHeightMeters = 1.8288; // 6 ft
    public static final double kShooterExitHeightMeters = 0.90;

    public static final double kMaxApexHeightMeters = 3.048; // 10 ft
    public static final double kAngleStepDeg = 0.5;

    public static final double kMinExitSpeedMps = 3.0;
    public static final double kMaxExitSpeedMps = 25.0;

    public static final Translation2d kFallbackScoreXY = kBlueScoreXY;
  }

  private final ShooterSubsystem shooter;
  private final HoodSubsystem hood;
  private final Supplier<Translation2d> robotXYSupplier;

  public AimHoodAndShooter(
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      Supplier<Translation2d> robotXYSupplier) {

    this.shooter = shooter;
    this.hood = hood;
    this.robotXYSupplier = robotXYSupplier;
    addRequirements(shooter, hood);
  }

  @Override
  public void execute() {
    Translation2d robotXY = robotXYSupplier.get();
    Translation2d targetXY = getAllianceTarget();

    double x = robotXY.getDistance(targetXY);
    double y = Constants.kTargetHeightMeters - Constants.kShooterExitHeightMeters;

    Solution sol = findBestSolution(x, y);
    if (sol == null) {
      shooter.stop();
      return;
    }

    hood.setAngleDeg(sol.angleDeg);
    shooter.setFlywheelRPM(
        ShooterSubsystem.exitSpeedMpsToFlywheelRPM(sol.exitSpeedMps));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // ---------------- internal helpers ----------------

  private static class Solution {
    final double angleDeg;
    final double exitSpeedMps;

    Solution(double angleDeg, double exitSpeedMps) {
      this.angleDeg = angleDeg;
      this.exitSpeedMps = exitSpeedMps;
    }
  }

  private Solution findBestSolution(double xMeters, double yMeters) {
    double bestV = Double.POSITIVE_INFINITY;
    double bestAngle = Double.NaN;

    for (double angleDeg = HoodSubsystem.HoodConstants.kMinAngleDeg;
         angleDeg <= HoodSubsystem.HoodConstants.kMaxAngleDeg;
         angleDeg += Constants.kAngleStepDeg) {

      double theta = Math.toRadians(angleDeg);
      double v = ShooterMath.requiredExitSpeed(xMeters, yMeters, theta);
      if (!Double.isFinite(v)) continue;

      if (v < Constants.kMinExitSpeedMps || v > Constants.kMaxExitSpeedMps) continue;

      double apex = ShooterMath.apexHeight(
          Constants.kShooterExitHeightMeters, v, theta);

      if (apex > Constants.kMaxApexHeightMeters) continue;

      if (v < bestV) {
        bestV = v;
        bestAngle = angleDeg;
      }
    }

    if (!Double.isFinite(bestV)) return null;
    return new Solution(bestAngle, bestV);
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
}
