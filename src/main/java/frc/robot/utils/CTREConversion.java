package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;

public class CTREConversion {
  // Positional conversions
  public static double falconToDegrees(double ticks, double gearRatio) {
    return falconToDegrees((int) ticks, gearRatio);
  }

  public static double falconToDegrees(int ticks, double gearRatio) {
    return ticks * Constants.CTREConstants.FALCON_TO_DEG / gearRatio;
  }

  public static int degreesToFalcon(double degrees, double gearRatio) {
    return (int) (degrees * Constants.CTREConstants.DEG_TO_FALCON * gearRatio);
  }

  public static double falconToRadians(int ticks, double gearRatio) {
    return ticks * Constants.CTREConstants.FALCON_TO_RAD / gearRatio;
  }

  public static Rotation2d falconToRotation2d(double ticks, double gearRatio) {
    return falconToRotation2d((int) ticks, gearRatio);
  }

  public static Rotation2d falconToRotation2d(int ticks, double gearRatio) {
    return Rotation2d.fromRadians(ticks * Constants.CTREConstants.FALCON_TO_RAD / gearRatio);
  }

  public static int radiansToFalcon(double radians, double gearRatio) {
    return (int) (radians * Constants.CTREConstants.RAD_TO_FALCON * gearRatio);
  }

  public static double degreesToCANCoder(double degrees, double gearRatio) {
    return degrees * Constants.CTREConstants.DEG_TO_CANCODER * gearRatio;
  }

  public static double falconToMeters(double ticks, double circumference_M, double gearRatio) {
    return falconToMeters((int) ticks, circumference_M, gearRatio);
  }

  public static double falconToMeters(int ticks, double circumference_M, double gearRatio) {
    return ticks * Constants.CTREConstants.FALCON_TICKS_TO_ROT * circumference_M / gearRatio;
  }

  public static int metersToFalcon(double meters, double circumference_M, double gearRatio) {
    return (int
    ) (meters * Constants.CTREConstants.ROT_TO_FALCON_TICKS * gearRatio / circumference_M);
  }

  public static double nativeToHeightMeters(
      double ticks, double circumference_M, double gearRatio
  ) {
    return nativeToHeightMeters((int) ticks, circumference_M, gearRatio);
  }

  public static double nativeToHeightMeters(int ticks, double circumference_M, double gearRatio) {
    return ticks * circumference_M / (Constants.CTREConstants.FALCON_TICKS_TO_ROT * gearRatio);
  }

  public static double heightMetersToNative(
      double height_M, double circumference_M, double gearRatio
  ) {
    return height_M * Constants.CTREConstants.FALCON_ENCODER_TICKS * gearRatio / circumference_M;
  }

  // Rotational rate conversions (time unit of minutes)
  public static double falconToRPM(double tickVelocity, double gearRatio) {
    return tickVelocity * Constants.CTREConstants.FALCON_TO_RPM / gearRatio;
  }

  public static int RPMToFalcon(double rpm, double gearRatio) {
    return (int) (rpm * Constants.CTREConstants.RPM_TO_FALCON * gearRatio);
  }

  public static double RPMToCANCoder(double rpm, double gearRatio) {
    // RPM --> cancoder RPM --> cancoder ticks per 100ms
    return rpm * gearRatio * Constants.CTREConstants.RPM_TO_CANCODER;
  }

  // Translational rate conversions (time unit of seconds)
  public static double falconToMPS(double tickVelocity, double circumference_M, double gearRatio) {
    // ticks per 100ms --> motor RPS --> wheel/drum RPS --> wheel drum surface speed MPS
    return tickVelocity * Constants.CTREConstants.FALCON_TO_RPS * circumference_M / gearRatio;
  }

  public static double MPSToFalcon(double velocity_MPS, double circumference_M, double gearRatio) {
    // RPS_TO_FALCON units = (ticks / 100ms) / (motor rotations/sec)
    // MPS / C --> RPS --> * gear ratio --> motor RPS
    return Constants.CTREConstants.RPS_TO_FALCON * velocity_MPS * gearRatio / circumference_M;
  }
}
