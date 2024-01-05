package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class VelocityLimits2d {
  public double maxTranslationalVelocityMPS;
  public Rotation2d maxAngularVelocityPS;

  public VelocityLimits2d(double maxTranslationalVelocityMPS, Rotation2d maxAngularVelocityPS) {
    this.maxTranslationalVelocityMPS = maxTranslationalVelocityMPS;
    this.maxAngularVelocityPS = maxAngularVelocityPS;
  }

  public static VelocityLimits2d fromRadians(
      double maxTranslationalVelocityMPS, double maxAngularVelocityRADPS
  ) {
    return new VelocityLimits2d(
        maxTranslationalVelocityMPS, Rotation2d.fromRadians(maxAngularVelocityRADPS)
    );
  }

  public static VelocityLimits2d fromDegrees(
      double maxTranslationalVelocityMPS, double maxAngularVelocityDPS
  ) {
    return new VelocityLimits2d(
        maxTranslationalVelocityMPS, Rotation2d.fromDegrees(maxAngularVelocityDPS)
    );
  }

  public void multiply(double c) {
    maxTranslationalVelocityMPS *= c;
    maxAngularVelocityPS = maxAngularVelocityPS.times(c);
  }
}
