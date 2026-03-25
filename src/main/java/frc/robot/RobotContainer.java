package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  public final HoodSubsystem m_hoodSubsytem = new HoodSubsystem();
  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final Map<String, Command> autos = new HashMap<>();

  public RobotContainer() {
    configureBindings();
    configureAutos();
  }

  

  
    private void configureAutos() {
        NamedCommands.registerCommand("Start Intake", new Pickup(m_intakeSubsystem, m_conveyorSubsystem));
        NamedCommands.registerCommand("Stop Intake", new IntakeStop(m_intakeSubsystem, m_conveyorSubsystem));
        NamedCommands.registerCommand("Drop Intake", new DropIntake(m_intakePivotSubsystem));

        NamedCommands.registerCommand("Spin Up Shooter", new InstantCommand(() -> m_shooterSubsystem.setFlywheelRPM(6000), m_shooterSubsystem));
            
        NamedCommands.registerCommand("Shoot Balls", new FeedBall(m_indexerSubsystem, m_conveyorSubsystem));

        NamedCommands.registerCommand("Stop All", new StopAll(m_conveyorSubsystem, m_indexerSubsystem, m_shooterSubsystem, m_intakeSubsystem));

        // ===== Auto 1 =====
        AutoRoutine newPathRoutine = autoFactory.newRoutine("NewPathAuto");
        AutoTrajectory newPath = newPathRoutine.trajectory("NewPath");
        
        newPathRoutine.active().onTrue(
            Commands.sequence(
                newPath.resetOdometry(),
                newPath.cmd()
            )
        );

        
        // ===== Auto 2 =====
        //bump
        AutoRoutine testRoutine = autoFactory.newRoutine("TestAuto");
        AutoTrajectory testPath = testRoutine.trajectory("Test");
        AutoTrajectory testPath2 = testRoutine.trajectory("TestCont");
        testRoutine.active().onTrue(
            Commands.sequence(
                testPath.resetOdometry(),
                testPath.cmd()
            )
        );
        testPath.done().onTrue(
            testPath2.cmd()
        );

        //trench
        AutoRoutine testRoutineTrench = autoFactory.newRoutine("TestAuto");
        AutoTrajectory testPathTrench1 = testRoutineTrench.trajectory("Test");
        AutoTrajectory testPathTrench2 = testRoutineTrench.trajectory("TestCont2");
        testRoutineTrench.active().onTrue(
            Commands.sequence(
                testPathTrench1.resetOdometry(),
                testPathTrench1.cmd()
            )
        );
        testPathTrench1.done().onTrue(
            testPathTrench2.cmd()
        );
        // ===== Put them in chooser =====
        autoChooser.setDefaultOption("NewPath Auto", newPathRoutine.cmd());
        autoChooser.addOption("Test Auto Bump", testRoutine.cmd());
        autoChooser.addOption("Test Auto Trench", testRoutineTrench.cmd());

        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        .whileTrue(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(5000)))
        .onFalse(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(0)));

    m_driverController.a()
        .whileTrue(new Pickup(m_intakeSubsystem, m_conveyorSubsystem));

    m_driverController.rightBumper()
        .whileTrue(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(6000)))
        .onFalse(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(0)));

    m_driverController.rightTrigger()
        .whileTrue(new AimAtHubBack(drivetrain, () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX()));
    m_driverController.povUp()
        .whileTrue(m_hoodSubsystem.runOnce(() -> m_hoodSubsystem.setSpeed(.5)))
        .onFalse(m_hoodSubsystem.runOnce(() -> m_hoodSubsystem.stop()));
    m_driverController.povRight()
        .whileTrue(m_hoodSubsystem.runOnce(() -> m_hoodSubsystem.setSpeed(-.5)))
        .onFalse(m_hoodSubsystem.runOnce(() -> m_hoodSubsystem.stop()));
    m_driverController.rightTrigger()
        .whileTrue(new SetShooterSpeedFromPosition(
            m_shooterSubsystem,
            m_conveyorSubsystem,
            m_indexerSubsystem,
            () -> drivetrain.getState().Pose.getTranslation()
        ));







    }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}