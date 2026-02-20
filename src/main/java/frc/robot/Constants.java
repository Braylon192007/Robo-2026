// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  public static class HoodConstants {

    // Servo Hub (CAN)
    public static final int kServoHubCanId = 4; // <-- change if needed

    // Two linear servos, same mechanism
    public static final ChannelId kHoodChannelIdLeft  = ChannelId.kChannelId0;
    public static final ChannelId kHoodChannelIdRight = ChannelId.kChannelId1;

    // Pulse widths (microseconds)
    public static final int kMinPulseUs    = 900;
    public static final int kCenterPulseUs = 1262;
    public static final int kMaxPulseUs    = 1625;

    // Stroke range (mm) of your actuator
    public static final double kMinStrokeMm = 0.0;
    public static final double kMaxStrokeMm = 140.0;

    // If you map distance -> angle later (optional)
    public static final double kMinAngleDeg = 55.0;
    public static final double kMaxAngleDeg = 80.0;

    // If one actuator is mirrored mechanically, you may need to invert one side.
    // Start false. If one side moves opposite, set true.
    public static final boolean kInvertRightPulse = false;

    // Dashboard / logic tolerance (no real feedback unless you add it)
    public static final double kStrokeToleranceMm = 1.0;
  }

  


  public static class ShooterConstants {
    // Hardware
    public static final int kShooterMotorCAN = 7; // <-- change
    public static final boolean kMotorInverted = false;

    // Flywheel geometry
    public static final double kFlywheelDiameterMeters = 0.1016; // 4 in
    public static final double kFlywheelRadiusMeters = kFlywheelDiameterMeters / 2.0;

    // Gear ratio: motorRotations * kGearRatio = flywheelRotations
    public static final double kGearRatio = 1.0;

    // Exit speed ~= wheel surface speed * this
    public static final double kExitSpeedPerWheelSpeed = 0.85; // tune

    // Limits
    public static final double kMinRPM = 0;
    public static final double kMaxRPM = 6000; // tune safe max

    // Closed-loop (starter values; tune)
    public static final double kP = 0.00025;
    public static final double kI = 0.0;
    public static final double kD = 0.00015;

    // NOTE: velocityFF() is deprecated in REVLib in favor of feedForward config,
    // but it still works on most 2025 builds. If it errors, tell me and I’ll swap to feedForward.
    public static final double kFF = 0.00017;

    // Ready tolerance
    public static final double kRPMTolerance = 75.0;
  }
  public static class VisionConstants {
    public static final String kLLFront = "limelight-front";
    public static final String kLLLeft  = "limelight-left";
    public static final String kLLRight = "limelight-right";

    /** If robot is spinning faster than this, ignore vision updates (deg/sec). */
    public static final double kMaxYawRateDegPerSec = 720.0; // common rule of thumb :contentReference[oaicite:3]{index=3}

    /** Heading std dev (rad). Keep large to mostly trust gyro for heading. :contentReference[oaicite:4]{index=4} */
    public static final double kThetaStdRad = 999.0;

    /** Base XY std dev (meters) when tags are good. Tune. */
    public static final double kBaseXYStd = 0.15;

    /** How much to trust measurements as tag distance grows (bigger = trust less). Tune. */
    public static final double kDistancePenalty = 0.08;

    /** Extra trust bonus for seeing multiple tags (bigger = trust more). Tune. */
    public static final double kMultiTagBonus = 0.7;
  }
  public static class AimHoodAndShooterConstants {
    public static final Translation2d kBlueScoreXY = new Translation2d(4.637, 4.072);
    public static final Translation2d kRedScoreXY  = new Translation2d(11.984, 4.072);

    public static final double kTargetHeightMeters = 1.8288; // 6 ft
    public static final double kShooterExitHeightMeters = 0.4826;

    public static final double kMaxApexHeightMeters = 2.4384; // 8 ft
    public static final double kAngleStepDeg = 0.5;

    public static final double kMinExitSpeedMps = 3.0;
    public static final double kMaxExitSpeedMps = 25.0;

    public static final Translation2d kFallbackScoreXY = kBlueScoreXY;
  }
  public static class SetShooterSpeedFromPositionConstants {
    public static final Translation2d kBlueScoreXY = new Translation2d(4.637, 4.072);
    public static final Translation2d kRedScoreXY  = new Translation2d(11.984, 4.072);

    public static final double kTargetHeightMeters = 1.8288; // 6 ft
    public static final double kShooterExitHeightMeters = 0.5334; // measure this

    public static final double kMinExitSpeedMps = 3.0;
    public static final double kMaxExitSpeedMps = 25.0;

    public static final Translation2d kFallbackScoreXY = kBlueScoreXY;
  }
  public static class IntakeConstants {
    public static final int kIntakeMotorCAN = 3; // <-- change
    public static final boolean kInverted = false;

    // Speeds (tune on robot)
    public static final double kIntakeSpeed = 0.9;   // fast pull-in
    public static final double kOuttakeSpeed = -0.9; // fast spit-out

    // Safety
    public static final int kCurrentLimit = 30; // NEO 550 safe-ish limit
  }
  public static class ConveyorConstants {
    public static final int kConveyorMotorCAN = 2; // <-- change
    public static final boolean kInverted = false;

    // Speeds (tune on robot)
    public static final double kFeedSpeed = 0.90;     // hopper -> indexer
    public static final double kReverseSpeed = -0.90; // clear jams / eject

    // Current limit (NEO Vortex is strong — limit it)
    public static final int kCurrentLimit = 50;

    // Optional ramp rate (keeps it snappy but less violent)
    public static final double kOpenLoopRampRate = 0.05;
  }
  public static class IndexerConstants {
    public static final int kIndexerMotorCAN = 5; // <-- change
    public static final boolean kInverted = false;

    // Speeds (tune)
    public static final double kFeedSpeed = 1;     // up into shooter (compression load)
    public static final double kReverseSpeed = -1; // clear jam / back ball out

    // Current limiting helps a LOT with compression/jams
    public static final int kCurrentLimit = 100;

    // Optional ramp so it doesn't spike hard into compression
    public static final double kOpenLoopRampRate = 0.08;
  }
  public static class PivotConstants {

  // CAN ID
  public static final int kPivotMotorCAN = 6;

  // Motor direction
  public static final boolean kMotorInverted = false;

  // Angle limits (degrees)
  // Define UP as 0 deg
  public static final double kUpDeg = 0.0;
  public static final double kDownDeg = 80.0;

  public static final double kMinDeg = kUpDeg;
  public static final double kMaxDeg = kDownDeg;

  /*
   * Conversion
   * 25:1 reduction
   * 360 deg per output revolution
   */
  public static final double kMotorRotPerDeg = 25.0 / 360.0; // ≈ 0.06944

  /*
   * MAXMotion limits
   * Units: MOTOR rotations / second
   */
  public static final double kCruiseRotPerSec = 12.0;    // ~720 RPM motor
  public static final double kAccelRotPerSec2 = 60.0;    // snappy return

  // Allowed error before MAXMotion "gives up" on profiling
  public static final double kAllowedProfileErrorRot = 0.08;

  /*
   * PID (starting values – expect to tune kP slightly)
   */
  public static final double kP = 0.25;
  public static final double kI = 0.0;
  public static final double kD = 2.0;

  /*
   * Feedforward
   * Set to 0 unless you add gravity compensation
   */
  public static final double kFF = 0.0;

  // How close is "good enough"
  public static final double kToleranceDeg = 2.0;

  // Current limit for holding against impacts
  public static final int kCurrentLimitA = 25;
}

}
