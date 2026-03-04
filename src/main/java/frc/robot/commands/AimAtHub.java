package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AimAtHub extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier xSupplier; // joystick x (forward) [-1..1]
  private final DoubleSupplier ySupplier; // joystick y (left)    [-1..1]

  // ===== Driver translation shaping =====
  private static final double kDriveDeadband = 0.10;
  private static final double kHardStopStickMag = 0.05; // if stick mag < this, force vx/vy = 0

  // ===== Heading controller gains (less twitchy) =====
  private static final double kHeadingP = 6.0;
  private static final double kHeadingI = 0.0;
  private static final double kHeadingD = 0.08;

  // ===== CTRE facing-angle settings =====
  private static final double kMaxRotRateRadPerSec = Math.toRadians(540.0);
  private static final double kRotDeadbandRad = Math.toRadians(2.0);

  // ===== "Lock window" to stop hunting when basically on target =====
  private static final double kLockOnDeg = 2.0;   // enter lock when |error| < this
  private static final double kUnlockDeg = 4.0;   // leave lock when |error| > this (hysteresis)

  private boolean headingLocked = false;
  private Rotation2d lockedHeading = Rotation2d.kZero;

  private final SwerveRequest.FieldCentricFacingAngle request =
      new SwerveRequest.FieldCentricFacingAngle()
          // Velocity control holds 0 MUCH better than open-loop voltage (less jitter)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withHeadingPID(kHeadingP, kHeadingI, kHeadingD)
          .withMaxAbsRotationalRate(kMaxRotRateRadPerSec)
          .withRotationalDeadband(kRotDeadbandRad);

  public AimAtHub(CommandSwerveDrivetrain drivetrain,
                  DoubleSupplier xSupplier,
                  DoubleSupplier ySupplier) {
    this.drivetrain = drivetrain;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    addRequirements(drivetrain);
  }

  private static Translation2d getHubTarget() {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    return (alliance == DriverStation.Alliance.Red)
        ? Constants.AimHoodAndShooterConstants.kRedScoreXY
        : Constants.AimHoodAndShooterConstants.kBlueScoreXY;
  }

  @Override
  public void initialize() {
    headingLocked = false;
  }

  @Override
  public void execute() {
    var pose = drivetrain.getState().Pose;

    Translation2d robotXY = pose.getTranslation();
    Translation2d targetXY = getHubTarget();

    Translation2d toTarget = targetXY.minus(robotXY);
    double distanceM = toTarget.getNorm();

    // Base desired heading points from robot -> target in field coords
    Rotation2d desiredHeading = Rotation2d.fromRadians(
        Math.atan2(toTarget.getY(), toTarget.getX())
    );

    // You said +180 makes it face correctly for your setup
    Rotation2d desiredWithOffset = desiredHeading.plus(Rotation2d.fromDegrees(180.0));

    // ===== Joystick Processing (true deadband + curve + hard stop) =====
    double rawX = xSupplier.getAsDouble();
    double rawY = ySupplier.getAsDouble();

    // true deadband (NO rescale)
    double x = Math.abs(rawX) < kDriveDeadband ? 0.0 : rawX;
    double y = Math.abs(rawY) < kDriveDeadband ? 0.0 : rawY;

    // square for finer low-speed control
    x = Math.copySign(x * x, x);
    y = Math.copySign(y * y, y);

    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(
        edu.wpi.first.units.Units.MetersPerSecond
    );

    double vx = x * maxSpeed;
    double vy = y * maxSpeed;

    // HARD STOP if stick is basically centered (kills drift jitter)
    double stickMag = Math.hypot(rawX, rawY);
    if (stickMag < kHardStopStickMag) {
      vx = 0.0;
      vy = 0.0;
    }

    // ===== Heading lock window (hysteresis) =====
    Rotation2d currentHeading = pose.getRotation();
    Rotation2d error = desiredWithOffset.minus(currentHeading);
    double errorDeg = Math.abs(error.getDegrees());

    if (!headingLocked) {
      if (errorDeg < kLockOnDeg) {
        headingLocked = true;
        lockedHeading = currentHeading; // hold where we are to prevent micro-hunting
      }
    } else {
      if (errorDeg > kUnlockDeg) {
        headingLocked = false;
      }
    }

    Rotation2d targetHeading = headingLocked ? lockedHeading : desiredWithOffset;

    // ===== Debug =====
    SmartDashboard.putNumber("Aim/DistanceToHubM", distanceM);
    SmartDashboard.putNumber("Aim/DesiredHeadingDeg", desiredWithOffset.getDegrees());
    SmartDashboard.putNumber("Aim/RobotHeadingDeg", currentHeading.getDegrees());
    SmartDashboard.putNumber("Aim/HeadingErrorDeg", error.getDegrees());
    SmartDashboard.putBoolean("Aim/HeadingLocked", headingLocked);

    SmartDashboard.putNumber("Aim/RawX", rawX);
    SmartDashboard.putNumber("Aim/RawY", rawY);
    SmartDashboard.putNumber("Aim/DbX", x);
    SmartDashboard.putNumber("Aim/DbY", y);
    SmartDashboard.putNumber("Aim/vxCmd", vx);
    SmartDashboard.putNumber("Aim/vyCmd", vy);
    SmartDashboard.putNumber("Aim/StickMag", stickMag);

    // ===== Apply control =====
    drivetrain.setControl(
        request
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withTargetDirection(targetHeading)
    );
  }

  @Override
  public void end(boolean interrupted) {
    headingLocked = false;
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0));
  }

  @Override
  public boolean isFinished() {
    return false; // intended for whileTrue()
  }
}