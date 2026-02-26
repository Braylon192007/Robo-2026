// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.Telemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  // The robot's subsystems and commands are defined here...
  public final IntakePivotSubsystem m_intakePivotSubsystem = new IntakePivotSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  public final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  public final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
  public void setupAutoBuilder() {
        drivetrain.configureAutoBuilder();
        
        //drivetrain.registerTelemetry(logger::telemeterize);
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically

            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    m_driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
    //m_driverController.leftBumper()
    //.onTrue(m_hoodSubsystem.runOnce(() -> m_hoodSubsystem.setStrokeMm(120)));

    m_driverController.a()
    .onTrue(m_hoodSubsystem.runOnce(() -> m_hoodSubsystem.setStrokeMm(30)));


    m_driverController.x()
    .whileTrue(m_intakePivotSubsystem.runOnce(() -> m_intakePivotSubsystem.setPercent(.4)))
    .onFalse(m_intakePivotSubsystem.runOnce(() -> m_intakePivotSubsystem.stop()));

    m_driverController.b()
    .whileTrue(m_intakePivotSubsystem.runOnce(() -> m_intakePivotSubsystem.setPercent(-.4)))
    .onFalse(m_intakePivotSubsystem.runOnce(() -> m_intakePivotSubsystem.stop()));

    //m_driverController.rightTrigger()
    //.whileTrue(new AimAtHub(drivetrain, () -> -m_driverController.getLeftY() * MaxSpeed, () -> -m_driverController.getLeftX() * MaxSpeed));


    m_driverController.leftTrigger()
    .whileTrue(new FeedBall(m_indexerSubsystem, m_conveyorSubsystem));

    m_driverController.rightTrigger()
    .whileTrue(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(4800)))
    .onFalse(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.setFlywheelRPM(0)));
    m_driverController.rightBumper()
    .whileTrue(new IntakeIn(m_intakeSubsystem));

    //Actual Drive Commands
    /*
    m_driverController.a()
    .whileTrue(new Pickup(m_intakeSubsystem, m_conveyorSubsystem));

    m_driverController.rightTrigger()
    .whileTrue(new SetShooterSpeedFromPosition(m_shooterSubsystem, m_hoodSubsystem, () -> drivetrain.getState().Pose.getTranslation()));

    m_driverController.rightBumper()
    .whileTrue(new AimAtHub(drivetrain, () -> -m_driverController.getLeftY() * MaxSpeed, () -> -m_driverController.getLeftX() * MaxSpeed));

    m_driverController.leftTrigger()
    .whileTrue(new FeedBall(m_indexerSubsystem, m_conveyorSubsystem));

    m_driverController.povDown()
    .whileTrue(new ReverseIndexer(m_indexerSubsystem));

    m_driverController.povUp()
    .whileTrue(new ClimbUp(m_climberSubsystem));
    m_driverController.povRight()
    .whileTrue(new Climb(m_climberSubsystem));
    */
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String autoName) {
    Command autoCommand = AutoBuilder.buildAuto(autoName);
    // An example command will be run in autonomous
    return autoCommand;
  }
}
