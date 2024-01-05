package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class AlliancePose2d {
  public final Pose2d blue, red;

  public AlliancePose2d(Pose2d blue, Pose2d red) {
    this.blue = blue;
    this.red = red;
  }

  public Pose2d get(boolean isRed) {
    return isRed ? red : blue;
  }
}
