package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AimAtHub extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier vxSupplier; // field-relative m/s
  private final DoubleSupplier vySupplier; // field-relative m/s

  // Tune this
  private static final double kDriveDeadband = 0.1;

  private final SwerveRequest.FieldCentricFacingAngle request =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AimAtHub(CommandSwerveDrivetrain drivetrain,
                  DoubleSupplier vxSupplier,
                  DoubleSupplier vySupplier) {
    this.drivetrain = drivetrain;
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;
    addRequirements(drivetrain);
  }

  private static Translation2d getHubTarget() {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    return (alliance == DriverStation.Alliance.Red)
        ? Constants.AimHoodAndShooterConstants.kRedScoreXY
        : Constants.AimHoodAndShooterConstants.kBlueScoreXY;
  }

  @Override
  public void execute() {
    var pose = drivetrain.getState().Pose;
    Translation2d robotXY = pose.getTranslation();
    Translation2d targetXY = getHubTarget();

    Translation2d toTarget = targetXY.minus(robotXY);
    double distanceM = toTarget.getNorm();

    Rotation2d desiredHeading = Rotation2d.fromRadians(
        Math.atan2(toTarget.getY(), toTarget.getX())
    );

    // Driver translation with deadband (keep sign, zero small noise)
    double vx = MathUtil.applyDeadband(vxSupplier.getAsDouble(), kDriveDeadband);
    double vy = MathUtil.applyDeadband(vySupplier.getAsDouble(), kDriveDeadband);

    SmartDashboard.putNumber("Aim/RobotX", pose.getX());
    SmartDashboard.putNumber("Aim/RobotY", pose.getY());
    SmartDashboard.putNumber("Aim/DistanceToHubM", distanceM);
    SmartDashboard.putNumber("Aim/DesiredHeadingDeg", desiredHeading.getDegrees());

    drivetrain.setControl(
        request
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withTargetDirection(desiredHeading)
    );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        request.withVelocityX(0).withVelocityY(0)
    );
  }

  @Override
  public boolean isFinished() {
    return false; // whileTrue()
  }
}
