package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

// TODO: Move all into a relevant class
public class CTREModuleState {
  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within -180 to 180 scope
   */
  public static double placeInClosestScopeDeg(double currentAngle, double desiredAngle) {
    double delta = (desiredAngle - currentAngle) % 360;
    if (delta > 180.0) {
      delta += -360.0;
    } else if (delta < -180.0) {
      delta += 360.0;
    }
    return currentAngle + delta;
  }

  public static double placeInClosestScopeRad(double scopeReference_rad, double newAngle_rad) {
    return Units.degreesToRadians(placeInClosestScopeDeg(
        Units.radiansToDegrees(scopeReference_rad), Units.radiansToDegrees(newAngle_rad)
    ));
  }

  public static SwerveModuleState optimize254(
      SwerveModuleState desiredState, Rotation2d currentAngle
  ) {
    // Place in closest scope
    double delta = (desiredState.angle.getDegrees() - currentAngle.getDegrees()) % 360;
    if (delta > 180.0) {
      delta += -360.0;
    } else if (delta < -180.0) {
      delta += 360.0;
    }
    double targetAngle_deg = currentAngle.getDegrees() + delta;
    double targetSpeed_mps = desiredState.speedMetersPerSecond;
    return new SwerveModuleState(targetSpeed_mps, Rotation2d.fromDegrees(targetAngle_deg));
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle
  ) {
    // Place in closest scope
    double delta = (desiredState.angle.getDegrees() - currentAngle.getDegrees()) % 360;
    if (delta > 180.0) {
      delta += -360.0;
    } else if (delta < -180.0) {
      delta += 360.0;
    }
    double targetAngle_deg = currentAngle.getDegrees() + delta;

    // Actual optimize
    double targetSpeed_mps = desiredState.speedMetersPerSecond;
    if (delta > 90.0) {
      targetSpeed_mps = -targetSpeed_mps;
      targetAngle_deg += -180.0;
    } else if (delta < -90.0) {
      targetSpeed_mps = -targetSpeed_mps;
      targetAngle_deg += 180.0;
    }
    return new SwerveModuleState(targetSpeed_mps, Rotation2d.fromDegrees(targetAngle_deg));
  }
}
