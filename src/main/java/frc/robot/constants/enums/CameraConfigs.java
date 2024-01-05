package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public enum CameraConfigs {
  // TODO: make these unique to device ids
  // TODO: measure drive base cam dimensions
  COMP_LEFT_CAMERA("Arducam_Forward", 0, 10.3, 7.5, 29.6, 16.0, 15.0, 0.0, "LEFT"),
  COMP_RIGHT_CAMERA("Arducam_Backward", 1, 10.3, -7.5, 29.6, -16.0, 15.0, 0.0, "RIGHT"),
  PRACTICE_LEFT_CAMERA("Arducam_Forward", 0, 10.3, 5.2, 30.55, 0.0, 15.0, 0, "LEFT"),
  PRACTICE_RIGHT_CAMERA("Arducam_Backward", 1, 10.3, -5.2, 30.55, 0.0, 15.0, 0, "RIGHT"),
  DRIVE_BASE_CAMERA("Arducam_Forward", 0, 1, -1, 29.75, 0.0, 21.0, 0.0, "LEFT");

  public final String cameraName;
  public final int id;
  public final String fieldObjectName;
  public final double r2cXaxisM, r2cYaxisM, r2cZaxisM;
  public final double r2cYaw_RAD, r2cPitch_RAD, r2cRoll_RAD;
  public final Translation3d r2cTranslation;
  public final Rotation3d r2cRotation;
  public final Transform3d r2cTransform;

  CameraConfigs(
      String cameraName,
      int id,
      double x,
      double y,
      double z,
      double yaw,
      double pitch,
      double roll,
      String fieldObjectName
  ) {
    this.cameraName = cameraName;
    this.id = id;
    this.fieldObjectName = fieldObjectName;
    r2cXaxisM = Units.inchesToMeters(x);
    r2cYaxisM = Units.inchesToMeters(y);
    r2cZaxisM = Units.inchesToMeters(z);
    r2cYaw_RAD = Units.degreesToRadians(yaw);
    r2cPitch_RAD = Units.degreesToRadians(pitch);
    r2cRoll_RAD = Units.degreesToRadians(roll);
    r2cTranslation = new Translation3d(r2cXaxisM, r2cYaxisM, r2cZaxisM);
    r2cRotation = new Rotation3d(r2cRoll_RAD, r2cPitch_RAD, r2cYaw_RAD);
    r2cTransform = new Transform3d(r2cTranslation, r2cRotation);
  }
}
