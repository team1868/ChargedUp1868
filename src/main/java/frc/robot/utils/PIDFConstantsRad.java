package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class PIDFConstantsRad extends PIDFConstants {
  public final Rotation2d tolerance;

  public PIDFConstantsRad(double p, double i, double d, Rotation2d tolerance) {
    this(p, i, d, 0.0, tolerance);
  }

  public PIDFConstantsRad(double p, double i, double d, double f, Rotation2d tolerance) {
    super(p, i, d, f);
    this.tolerance = tolerance;
  }

  public static PIDFConstantsRad fromRadians(
      double p, double i, double d, double f, double toleranceRad
  ) {
    return new PIDFConstantsRad(p, i, d, f, Rotation2d.fromRadians(toleranceRad));
  }

  public static PIDFConstantsRad fromDegrees(
      double p, double i, double d, double f, double toleranceDeg
  ) {
    return new PIDFConstantsRad(
        p,
        i,
        d,
        // Units.radiansToDegrees(p),
        // Units.radiansToDegrees(i),
        // Units.radiansToDegrees(d),
        f,
        Rotation2d.fromDegrees(toleranceDeg)
    );
  }
}
