package frc.robot.utils;

public class PIDFConstantsM extends PIDFConstants {
  public final double toleranceM;
  public PIDFConstantsM(double p, double i, double d, double toleranceM) {
    this(p, i, d, 0.0, toleranceM);
  }

  public PIDFConstantsM(double p, double i, double d, double f, double toleranceM) {
    super(p, i, d, f);
    this.toleranceM = toleranceM;
  }
}
