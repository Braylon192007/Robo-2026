package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
/**
 * Fuses multiple Limelights by producing a set of VisionObservations.
 *
 * REQUIREMENTS (Limelight side):
 * - Upload field map (.fmap)
 * - AprilTag pipeline with 3D enabled
 * - Set each Limelight's robot-space pose in LL web UI
 * - For MegaTag2: call SetRobotOrientation() every loop before reading pose
 */
public class VisionSubsystem extends SubsystemBase {

  /** Change these to your limelight NT table names. */
  

  /** Simple container for a vision measurement for pose estimator fusion. */
  public static class VisionObservation {
    public final Pose2d pose;
    public final double timestampSeconds;
    public final Matrix<N3, N1> stdDevs;

    public VisionObservation(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.stdDevs = stdDevs;
    }
  }

  private final String[] limelights = {
      VisionConstants.kLLFront
  };

  /**
   * Call this every loop from drivetrain (or wherever you have gyro info).
   *
   * @param robotYawDeg Current robot yaw in degrees (CCW+).
   * @param yawRateDegPerSec Current robot yaw rate in deg/sec.
   * @return array of valid observations (0..3)
   */
  public VisionObservation[] getVisionObservations(double robotYawDeg, double yawRateDegPerSec) {
    // MegaTag2 requires telling Limelight your current orientation before requesting pose. :contentReference[oaicite:5]{index=5}
    for (String name : limelights) {
      LimelightHelpers.SetRobotOrientation(name, robotYawDeg, 0.0, 0.0, yawRateDegPerSec, 0.0, 0.0);
    }

    // If spinning too fast, ignore all vision this cycle (prevents wild jumps). :contentReference[oaicite:6]{index=6}
    if (Math.abs(yawRateDegPerSec) > VisionConstants.kMaxYawRateDegPerSec) {
      return new VisionObservation[0];
    }

    java.util.ArrayList<VisionObservation> out = new java.util.ArrayList<>();

    for (String name : limelights) {
      LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name); // :contentReference[oaicite:7]{index=7}

      if (est == null) continue;
      if (est.tagCount <= 0) continue;

      Pose2d pose = est.pose;
      double ts = est.timestampSeconds;

      // est.avgTagDist is in meters (Limelight NT botpose docs). :contentReference[oaicite:8]{index=8}
      double xyStd = computeXYStdDevs(est.tagCount, est.avgTagDist);

      // Trust gyro for heading: make theta std dev huge. :contentReference[oaicite:9]{index=9}
      Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, VisionConstants.kThetaStdRad);

      out.add(new VisionObservation(pose, ts, stdDevs));
    }

    return out.toArray(new VisionObservation[0]);
  }

  /**
   * Dynamic trust model:
   * - More tags => smaller std dev
   * - Further tags => larger std dev
   */
  private static double computeXYStdDevs(int tagCount, double avgTagDistMeters) {
    // Base grows with distance
    double std = VisionConstants.kBaseXYStd + VisionConstants.kDistancePenalty * avgTagDistMeters;

    // Seeing multiple tags boosts confidence
    if (tagCount >= 2) {
      std *= (1.0 - VisionConstants.kMultiTagBonus); // e.g. 0.3x if bonus=0.7
    } else {
      // Single tag -> less confident (keep as-is)
    }

    // Clamp so it never goes insanely small
    return Math.max(std, 0.05);
  }
}
