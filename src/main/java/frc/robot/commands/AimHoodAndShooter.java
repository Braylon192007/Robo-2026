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
import frc.robot.Constants.AimHoodAndShooterConstants;
import frc.robot.Constants.HoodConstants;
public class AimHoodAndShooter extends Command {

  

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
    double y = AimHoodAndShooterConstants.kTargetHeightMeters - AimHoodAndShooterConstants.kShooterExitHeightMeters;

    Solution sol = findBestSolution(x, y);
    if (sol == null) {
      shooter.stop();
      return;
    }

    hood.setStrokeMm(sol.strokeMm);
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
    final double strokeMm;
    final double exitSpeedMps;

    Solution(double strokeMm, double exitSpeedMps) {
      this.strokeMm = strokeMm;
      this.exitSpeedMps = exitSpeedMps;
    }
  }

  private Solution findBestSolution(double xMeters, double yMeters) {
    double bestV = Double.POSITIVE_INFINITY;
    double bestStroke = Double.NaN;

    for (double strokeMm = HoodConstants.kMinStrokeMm;
         strokeMm <= HoodConstants.kMaxStrokeMm;
         strokeMm += AimHoodAndShooterConstants.kAngleStepDeg) {

      double theta = Math.toRadians(strokeMm);
      double v = ShooterMath.requiredExitSpeed(xMeters, yMeters, theta);
      if (!Double.isFinite(v)) continue;

      if (v < AimHoodAndShooterConstants.kMinExitSpeedMps || v > AimHoodAndShooterConstants.kMaxExitSpeedMps) continue;

      double apex = ShooterMath.apexHeight(
          AimHoodAndShooterConstants.kShooterExitHeightMeters, v, theta);

      if (apex > AimHoodAndShooterConstants.kMaxApexHeightMeters) continue;

      if (v < bestV) {
        bestV = v;
        bestStroke = strokeMm;
      }
    }

    if (!Double.isFinite(bestV)) return null;
    return new Solution(bestStroke, bestV);
  }

  private static Translation2d getAllianceTarget() {
    Optional<Alliance> a = DriverStation.getAlliance();
    if (a.isPresent()) {
      return (a.get() == Alliance.Red)
          ? AimHoodAndShooterConstants.kRedScoreXY
          : AimHoodAndShooterConstants.kBlueScoreXY;
    }
    return AimHoodAndShooterConstants.kFallbackScoreXY;
  }
}
