package frc.robot.utils;

import edu.wpi.first.math.util.Units;

public class UnitUtils {
  public static double[] inchesToMeters(double[] inches) {
    double[] res = new double[inches.length];
    for (int i = 0; i < inches.length; i++) {
      res[i] = Units.inchesToMeters(inches[i]);
    }
    return res;
  }
}
