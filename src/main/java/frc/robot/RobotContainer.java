package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import java.util.HashMap;
import java.util.Map;

import frc.robot.generated.TunerConstants;
import frc.robot.Telemetry;

public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final IntakePivotSubsystem m_intakePivotSubsystem = new IntakePivotSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  public final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final Map<String, Command> autos = new HashMap<>();

  public RobotContainer() {
    registerNamedCommands();
    configureBindings();
    buildAutos();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("IntakeDown",
        new InstantCommand(() -> m_intakePivotSubsystem.setPercent(-.4), m_intakePivotSubsystem));

    NamedCommands.registerCommand("IntakeStop",
        new InstantCommand(() -> m_intakePivotSubsystem.stop(), m_intakePivotSubsystem));

    NamedCommands.registerCommand("Intake",
        new Pickup(m_intakeSubsystem, m_conveyorSubsystem));

    NamedCommands.registerCommand("StopIntake",
        new InstantCommand(() -> {
          m_intakeSubsystem.stop();
          m_conveyorSubsystem.stop();
        }, m_intakeSubsystem, m_conveyorSubsystem));

    NamedCommands.registerCommand("Charge",
        new InstantCommand(() -> m_shooterSubsystem.setFlywheelRPM(4500), m_shooterSubsystem));

    NamedCommands.registerCommand("StopShooter",
        new InstantCommand(() -> {
          m_shooterSubsystem.setFlywheelRPM(0);
          m_conveyorSubsystem.stop();
          m_indexerSubsystem.stop();
        }, m_shooterSubsystem, m_conveyorSubsystem, m_indexerSubsystem));

    NamedCommands.registerCommand("ConveyorGo",
        new InstantCommand(() -> m_conveyorSubsystem.feed(), m_conveyorSubsystem));

    NamedCommands.registerCommand("Feed",
        new InstantCommand(() -> {
          m_indexerSubsystem.feed();
          m_conveyorSubsystem.feed();
        }, m_indexerSubsystem, m_conveyorSubsystem));

    NamedCommands.registerCommand("StopFeed",
        new InstantCommand(() -> {
          m_indexerSubsystem.stop();
          m_conveyorSubsystem.stop();
        }, m_indexerSubsystem, m_conveyorSubsystem));
  }

  private void buildAutos() {
    autos.put("Test", AutoBuilder.buildAuto("Test"));
    autos.put("LeftAuto", AutoBuilder.buildAuto("LeftAuto"));
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    m_driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    m_driverController.x()
        .whileTrue(m_intakePivotSubsystem.runOnce(() -> m_intakePivotSubsystem.setPercent(.4)))
        .onFalse(m_intakePivotSubsystem.runOnce(() -> m_intakePivotSubsystem.stop()));

    m_driverController.b()
        .whileTrue(m_intakePivotSubsystem.runOnce(() -> m_intakePivotSubsystem.setPercent(-.4)))
        .onFalse(m_intakePivotSubsystem.runOnce(() -> m_intakePivotSubsystem.stop()));

    m_driverController.leftTrigger()
        .whileTrue(new FeedBall(m_indexerSubsystem, m_conveyorSubsystem));

    m_driverController.leftBumper()
        .whileTrue(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(3200)))
        .onFalse(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(0)));

    m_driverController.a()
        .whileTrue(new Pickup(m_intakeSubsystem, m_conveyorSubsystem));

    m_driverController.rightBumper()
        .whileTrue(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(3900)))
        .onFalse(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(0)));

    m_driverController.rightTrigger()
        .whileTrue(new AimAtHub(drivetrain, () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX()));

    m_driverController.rightTrigger()
        .whileTrue(new SetShooterSpeedFromPosition(
            m_shooterSubsystem,
            m_conveyorSubsystem,
            m_indexerSubsystem,
            () -> drivetrain.getState().Pose.getTranslation()
        ));

    }

  public Command getAutonomousCommand(String autoName) {
    return autos.get(autoName);
  }
}