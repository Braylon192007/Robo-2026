// frc/robot/commands/AimAtHub.java
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AimAtHub extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier vxSupplier; // field-relative m/s
    private final DoubleSupplier vySupplier; // field-relative m/s

    private final SwerveRequest.FieldCentricFacingAngle request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AimAtHub(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier
    ) {
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
        Translation2d target = getHubTarget();

        double dx = target.getX() - pose.getX();
        double dy = target.getY() - pose.getY();

        Rotation2d desiredHeading = new Rotation2d(Math.atan2(dy, dx));

        drivetrain.setControl(
            request
                .withVelocityX(vxSupplier.getAsDouble())
                .withVelocityY(vySupplier.getAsDouble())
                .withTargetDirection(desiredHeading)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            request
                .withVelocityX(0)
                .withVelocityY(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // use whileTrue()
    }
}
