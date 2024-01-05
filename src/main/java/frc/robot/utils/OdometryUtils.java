package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class OdometryUtils {
  public static boolean inRange(
      Pose2d curPose,
      Rotation2d yaw,
      Pose2d targetPose,
      double xyToleranceM,
      double thetaToleranceDeg
  ) {
    double thetaDiff = targetPose.getRotation().getDegrees() - yaw.getDegrees();
    return Math.abs(targetPose.getX() - curPose.getX()) <= xyToleranceM
        && Math.abs(targetPose.getY() - curPose.getY()) <= xyToleranceM
        && Math.abs(thetaDiff > 180 ? thetaDiff - 360 : thetaDiff) <= thetaToleranceDeg;
  }
}
