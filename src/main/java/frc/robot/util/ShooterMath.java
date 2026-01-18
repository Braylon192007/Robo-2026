package frc.robot.util;

public class ShooterMath {
  public static final double g = 9.80665;

  /** Required exit speed to hit (x,y) with launch angle theta. Returns NaN if impossible. */
  public static double requiredExitSpeed(double xMeters, double yMeters, double thetaRad) {
    double cos = Math.cos(thetaRad);
    double tan = Math.tan(thetaRad);

    double denom = 2.0 * cos * cos * (xMeters * tan - yMeters);
    if (denom <= 1e-9) return Double.NaN;
    return Math.sqrt((g * xMeters * xMeters) / denom);
  }

  /** Apex height above field given start height y0, exit speed v, and angle theta. */
  public static double apexHeight(double startHeightMeters, double v, double thetaRad) {
    double sin = Math.sin(thetaRad);
    return startHeightMeters + (v * v * sin * sin) / (2.0 * g);
  }
}
