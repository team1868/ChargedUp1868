package frc.robot.constants.enums;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public enum TagPositions {
  OFFICIAL_16H5_TAG_1(1, 610.77, 42.19, Consts.GRID_Z_IN, Consts.RED_YAW_DEG),
  OFFICIAL_16H5_TAG_2(2, 610.77, 108.19, Consts.GRID_Z_IN, Consts.RED_YAW_DEG),
  OFFICIAL_16H5_TAG_3(3, 610.77, 174.19, Consts.GRID_Z_IN, Consts.RED_YAW_DEG),
  OFFICIAL_16H5_TAG_4(4, 636.96, 265.74, Consts.SUBSTATION_Z_IN, Consts.RED_YAW_DEG),
  OFFICIAL_16H5_TAG_5(5, 14.25, 265.74, Consts.SUBSTATION_Z_IN, Consts.BLUE_YAW_DEG),
  OFFICIAL_16H5_TAG_6(6, 40.45, 174.19, Consts.GRID_Z_IN, Consts.BLUE_YAW_DEG),
  OFFICIAL_16H5_TAG_7(7, 40.45, 108.19, Consts.GRID_Z_IN, Consts.BLUE_YAW_DEG),
  OFFICIAL_16H5_TAG_8(8, 40.45, 42.19, Consts.GRID_Z_IN, Consts.BLUE_YAW_DEG),
  NASA_16H5_TAG_1(
      1, 610.77 + FieldDims.NASA_FIELD_DIMS.xDelta_IN, 42.19, Consts.GRID_Z_IN, Consts.RED_YAW_DEG
  ),
  NASA_16H5_TAG_2(
      2, 610.77 + FieldDims.NASA_FIELD_DIMS.xDelta_IN, 108.19, Consts.GRID_Z_IN, Consts.RED_YAW_DEG
  ),
  NASA_16H5_TAG_3(
      3, 610.77 + FieldDims.NASA_FIELD_DIMS.xDelta_IN, 174.19, Consts.GRID_Z_IN, Consts.RED_YAW_DEG
  ),
  NASA_16H5_TAG_4(
      4,
      FieldDims.NASA_FIELD_DIMS.x_IN - Consts.TAG_WIDTH_IN,
      265.74,
      Consts.SUBSTATION_Z_IN,
      Consts.RED_YAW_DEG
  ),
  NASA_16H5_TAG_5(
      5,
      Consts.TAG_WIDTH_IN,
      265.74 + FieldDims.NASA_FIELD_DIMS.yDelta_IN,
      Consts.SUBSTATION_Z_IN,
      Consts.BLUE_YAW_DEG
  ),
  NASA_16H5_TAG_6(
      6,
      Consts.TAG_WIDTH_IN,
      174.19 + FieldDims.NASA_FIELD_DIMS.yDelta_IN,
      Consts.GRID_Z_IN,
      Consts.BLUE_YAW_DEG
  ),
  NASA_16H5_TAG_7(
      7,
      Consts.TAG_WIDTH_IN,
      108.19 + FieldDims.NASA_FIELD_DIMS.yDelta_IN,
      Consts.GRID_Z_IN,
      Consts.BLUE_YAW_DEG
  ),
  NASA_16H5_TAG_8(
      8,
      Consts.TAG_WIDTH_IN,
      42.19 + FieldDims.NASA_FIELD_DIMS.yDelta_IN,
      Consts.GRID_Z_IN,
      Consts.BLUE_YAW_DEG
  ),
  CURIE_16H5_TAG_1(OFFICIAL_16H5_TAG_1),
  CURIE_16H5_TAG_2(OFFICIAL_16H5_TAG_2),
  CURIE_16H5_TAG_3(OFFICIAL_16H5_TAG_3),
  CURIE_16H5_TAG_4(OFFICIAL_16H5_TAG_4),
  CURIE_16H5_TAG_5(OFFICIAL_16H5_TAG_5),
  CURIE_16H5_TAG_6(OFFICIAL_16H5_TAG_6),
  CURIE_16H5_TAG_7(OFFICIAL_16H5_TAG_7),
  CURIE_16H5_TAG_8(OFFICIAL_16H5_TAG_8),
  SVR_16H5_TAG_1(OFFICIAL_16H5_TAG_1),
  SVR_16H5_TAG_2(OFFICIAL_16H5_TAG_2),
  SVR_16H5_TAG_3(OFFICIAL_16H5_TAG_3),
  SVR_16H5_TAG_4(OFFICIAL_16H5_TAG_4),
  SVR_16H5_TAG_5(OFFICIAL_16H5_TAG_5),
  SVR_16H5_TAG_6(OFFICIAL_16H5_TAG_6),
  SVR_16H5_TAG_7(OFFICIAL_16H5_TAG_7),
  SVR_16H5_TAG_8(OFFICIAL_16H5_TAG_8),
  CANADA_16H5_TAG_1(OFFICIAL_16H5_TAG_1),
  CANADA_16H5_TAG_2(OFFICIAL_16H5_TAG_2),
  CANADA_16H5_TAG_3(OFFICIAL_16H5_TAG_3),
  CANADA_16H5_TAG_4(OFFICIAL_16H5_TAG_4),
  CANADA_16H5_TAG_5(OFFICIAL_16H5_TAG_5),
  CANADA_16H5_TAG_6(OFFICIAL_16H5_TAG_6),
  CANADA_16H5_TAG_7(OFFICIAL_16H5_TAG_7),
  CANADA_16H5_TAG_8(OFFICIAL_16H5_TAG_8),
  ARIZONA_16H5_TAG_1(OFFICIAL_16H5_TAG_1),
  ARIZONA_16H5_TAG_2(OFFICIAL_16H5_TAG_2),
  ARIZONA_16H5_TAG_3(OFFICIAL_16H5_TAG_3),
  ARIZONA_16H5_TAG_4(OFFICIAL_16H5_TAG_4),
  ARIZONA_16H5_TAG_5(OFFICIAL_16H5_TAG_5),
  ARIZONA_16H5_TAG_6(OFFICIAL_16H5_TAG_6),
  ARIZONA_16H5_TAG_7(OFFICIAL_16H5_TAG_7),
  ARIZONA_16H5_TAG_8(OFFICIAL_16H5_TAG_8),
  CHEZY_16H5_TAG_1(OFFICIAL_16H5_TAG_1),
  CHEZY_16H5_TAG_2(OFFICIAL_16H5_TAG_2),
  CHEZY_16H5_TAG_3(OFFICIAL_16H5_TAG_3),
  CHEZY_16H5_TAG_4(OFFICIAL_16H5_TAG_4),
  CHEZY_16H5_TAG_5(OFFICIAL_16H5_TAG_5),
  CHEZY_16H5_TAG_6(OFFICIAL_16H5_TAG_6),
  CHEZY_16H5_TAG_7(OFFICIAL_16H5_TAG_7),
  CHEZY_16H5_TAG_8(OFFICIAL_16H5_TAG_8),
  CALGAMES_16H5_TAG_1(OFFICIAL_16H5_TAG_1),
  CALGAMES_16H5_TAG_2(OFFICIAL_16H5_TAG_2),
  CALGAMES_16H5_TAG_3(OFFICIAL_16H5_TAG_3),
  CALGAMES_16H5_TAG_4(OFFICIAL_16H5_TAG_4),
  CALGAMES_16H5_TAG_5(OFFICIAL_16H5_TAG_5),
  CALGAMES_16H5_TAG_6(OFFICIAL_16H5_TAG_6),
  CALGAMES_16H5_TAG_7(OFFICIAL_16H5_TAG_7),
  CALGAMES_16H5_TAG_8(OFFICIAL_16H5_TAG_8);

  public final int id;
  public final AprilTag aprilTag;

  TagPositions(int id, double x, double y, double z, double yaw, double xShift, double yShift) {
    this.id = id;

    double x_M = Units.inchesToMeters(x + xShift);
    double y_M = Units.inchesToMeters(y + yShift);
    double z_M = Units.inchesToMeters(z);
    double yaw_RAD = Units.degreesToRadians(yaw);

    aprilTag = new AprilTag(id, new Pose3d(x_M, y_M, z_M, new Rotation3d(0.0, 0.0, yaw_RAD)));
  }
  TagPositions(int id, double x, double y, double z, double yaw) {
    this(id, x, y, z, yaw, 0.0, 0.0);
  }
  TagPositions(TagPositions tag) {
    this(tag, 0.0, 0.0);
  }
  TagPositions(TagPositions pos, double xShift, double yShift) {
    this.id = pos.id;

    this.aprilTag = new AprilTag(
        pos.id,
        new Pose3d(
            pos.aprilTag.pose.getX() + Units.inchesToMeters(xShift),
            pos.aprilTag.pose.getY() + Units.inchesToMeters(yShift),
            pos.aprilTag.pose.getZ(),
            pos.aprilTag.pose.getRotation()
        )
    );
  }

  public static final class Consts {
    public static final double TAG_WIDTH_IN = 0.25,
                               TAG_WIDTH_M = Units.inchesToMeters(TAG_WIDTH_IN);
    public static final double BLUE_YAW_DEG = 0.0;
    public static final double RED_YAW_DEG = 180.0;
    public static final double GRID_Z_IN = 18.22, GRID_Z_M = Units.inchesToMeters(GRID_Z_IN);
    public static final double SUBSTATION_Z_IN = 27.375,
                               SUBSTATION_Z_M = Units.inchesToMeters(SUBSTATION_Z_IN);
  }
}
