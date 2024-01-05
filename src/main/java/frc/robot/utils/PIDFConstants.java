package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

public class PIDFConstants extends PIDConstants {
  public final double f;
  public PIDFConstants(double p, double i, double d, double f) {
    super(p, i, d);
    this.f = f;
  }

  public SlotConfiguration toCTRESlotConfiguration() {
    SlotConfiguration configured = new SlotConfiguration();
    configured.kP = p;
    configured.kI = i;
    configured.kD = d;
    configured.kF = f;
    return configured;
  }
}
