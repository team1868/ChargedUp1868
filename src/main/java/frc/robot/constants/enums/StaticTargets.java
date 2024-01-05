package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.AlliancePose2d;

public enum StaticTargets {
  CHARGING_TARGET(
      Zones.ROBOT_COMMUNITY,
      new AlliancePose2d(
          new Pose2d(Zones.CHARGING_STATION_ROBOT_BALANCE.redMax, Rotation2d.fromDegrees(0.0)),
          new Pose2d(Zones.CHARGING_STATION_ROBOT_BALANCE.blueMax, Rotation2d.fromDegrees(0.0))
      )
  ),
  INTAKE_TARGET(Zones.LOADING, HPIntakeStations.DOUBLE_STATION_INNER.pose);

  Zones zone;
  AlliancePose2d target;

  StaticTargets(Zones zone, Pose2d blue, Pose2d red) {
    this.zone = zone;
    this.target = new AlliancePose2d(blue, red);
  }

  StaticTargets(Zones zone, AlliancePose2d target) {
    this.zone = zone;
    this.target = target;
  }

  public AlliancePose2d getAllianceTarget(Pose2d cur) {
    return new AlliancePose2d(
        zone.inBlueZone(cur.getTranslation()) ? target.blue : cur,
        zone.inRedZone(cur.getTranslation()) ? target.red : cur
    );
  }

  public Pose2d getAllianceTarget(Pose2d cur, boolean isRed) {
    if (isRed) {
      return zone.inRedZone(cur.getTranslation()) ? target.red : cur;
    } else {
      return zone.inBlueZone(cur.getTranslation()) ? target.blue : cur;
    }
  }
}
