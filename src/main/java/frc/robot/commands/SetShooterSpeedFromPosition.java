package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.SetShooterSpeedFromPositionConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterSpeedFromPosition extends Command {

  private final ShooterSubsystem shooter;
  private final ConveyorSubsystem conveyor;
  private final IndexerSubsystem indexer;
  private final Supplier<Translation2d> robotXYSupplier;

  private static final double kShooterReadyToleranceRPM = 100.0;

  public SetShooterSpeedFromPosition(
      ShooterSubsystem shooter,
      ConveyorSubsystem conveyor,
      IndexerSubsystem indexer,
      Supplier<Translation2d> robotXYSupplier) {
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.indexer = indexer;
    this.robotXYSupplier = robotXYSupplier;

    addRequirements(shooter, conveyor, indexer);
  }

  @Override
  public void initialize() {
    conveyor.stop();
    indexer.stop();
  }

  @Override
  public void execute() {
    Translation2d robotXY = robotXYSupplier.get();
    Translation2d targetXY = getAllianceTarget();

    double distMeters = robotXY.getDistance(targetXY);
    double distInches = distMeters * 39.37007874;

    double targetRPM = rpmFromDistanceInches(distInches);
    shooter.setFlywheelRPM(targetRPM);
/* 
    boolean readyToShoot = shooter.atTargetRPM(targetRPM, kShooterReadyToleranceRPM);

    if (readyToShoot) {
      conveyor.feed();
      indexer.feed();
    } else {
      conveyor.stop();
      indexer.stop();
    }
*/
    SmartDashboard.putNumber("Shooter/DistanceMeters", distMeters);
    SmartDashboard.putNumber("Shooter/DistanceInches", distInches);
    SmartDashboard.putNumber("Shooter/RPMSetpoint", targetRPM);
    SmartDashboard.putNumber("Shooter/CurrentFlywheelRPM", shooter.getFlywheelRPM());
    //SmartDashboard.putBoolean("Shooter/ReadyToShoot", readyToShoot);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    conveyor.stop();
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static Translation2d getAllianceTarget() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Red
          ? SetShooterSpeedFromPositionConstants.kRedScoreXY
          : SetShooterSpeedFromPositionConstants.kBlueScoreXY;
    }
    return SetShooterSpeedFromPositionConstants.kFallbackScoreXY;
  }

  private static double rpmFromDistanceInches(double in) {
    if (in <= 94.0) return 6000.0;
    if (in >= 140.0) return 6000.0;

    if (in <= 120.0) {
      return lerp(94.0, 6000.0, 120.0, 6000.0, in);
    }

    return lerp(120.0, 6000.0, 140.0, 6000.0, in);
  }

  private static double lerp(double x0, double y0, double x1, double y1, double x) {
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }
}