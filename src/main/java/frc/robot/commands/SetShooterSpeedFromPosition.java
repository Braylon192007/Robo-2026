package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.SetShooterSpeedFromPositionConstants;

public class SetShooterSpeedFromPosition extends Command {

  private final ShooterSubsystem shooter;
  private final Supplier<Translation2d> robotXYSupplier;

  public SetShooterSpeedFromPosition(ShooterSubsystem shooter, Supplier<Translation2d> robotXYSupplier) {
    this.shooter = shooter;
    this.robotXYSupplier = robotXYSupplier;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Translation2d robotXY = robotXYSupplier.get();
    Translation2d targetXY = getAllianceTarget();

    double distMeters = robotXY.getDistance(targetXY);
    double distInches = distMeters * 39.37007874;

    double rpm = rpmFromDistanceInches(distInches);
    shooter.setFlywheelRPM(rpm);

    // Helpful for tuning
    SmartDashboard.putNumber("Shooter/DistanceMeters", distMeters);
    SmartDashboard.putNumber("Shooter/DistanceInches", distInches);
    SmartDashboard.putNumber("Shooter/RPMSetpoint", rpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }

  private static Translation2d getAllianceTarget() {
    Optional<Alliance> a = DriverStation.getAlliance();
    if (a.isPresent()) {
      return (a.get() == Alliance.Red)
          ? SetShooterSpeedFromPositionConstants.kRedScoreXY
          : SetShooterSpeedFromPositionConstants.kBlueScoreXY;
    }
    return SetShooterSpeedFromPositionConstants.kFallbackScoreXY;
  }

  /**
   * Your measured points:
   * 94 in -> 4100 RPM
   * 120 in -> 4450 RPM
   * 140 in -> 4750 RPM
   *
   * Linear interpolation between points; clamped outside range.
   */
  private static double rpmFromDistanceInches(double in) {
    // clamp below/above
    if (in <= 94.0) return 4250.0;
    if (in >= 140.0) return 4900.0;

    // segment 94 -> 120
    if (in <= 120.0) {
      return lerp(94.0, 4250.0, 120.0, 4500.0, in);
    }

    // segment 120 -> 140
    return lerp(120.0, 4500.0, 140.0, 4900.0, in);
  }

  private static double lerp(double x0, double y0, double x1, double y1, double x) {
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }
}