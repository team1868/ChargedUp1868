package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.utils.AlliancePose2d;

public enum HPIntakeStations {
  UNKNOWN_INTAKE_STATION(LedColors.OFF),
  DOUBLE_STATION_INNER(
      LedColors.BLUE,
      LocationMath.DOUBLE_STATION_BLUE_X_M - Constants.LOADING_POSE_OFFSET_M
          - Constants.CRobot.drive.dims.halfBumperLength_M,
      LocationMath.DOUBLE_STATION_INNER_Y_M,
      LocationMath.DOUBLE_STATION_BLUE_YAW,
      LocationMath.DOUBLE_STATION_RED_X_M + Constants.LOADING_POSE_OFFSET_M
          + Constants.CRobot.drive.dims.halfBumperLength_M,
      LocationMath.DOUBLE_STATION_INNER_Y_M,
      LocationMath.DOUBLE_STATION_RED_YAW
  ),
  DOUBLE_STATION_OUTER(
      LedColors.GREEN,
      LocationMath.DOUBLE_STATION_BLUE_X_M - Constants.LOADING_POSE_OFFSET_M
          - Constants.CRobot.drive.dims.halfBumperLength_M,
      LocationMath.DOUBLE_STATION_OUTER_Y_M,
      LocationMath.DOUBLE_STATION_BLUE_YAW,
      LocationMath.DOUBLE_STATION_RED_X_M + Constants.LOADING_POSE_OFFSET_M
          + Constants.CRobot.drive.dims.halfBumperLength_M,
      LocationMath.DOUBLE_STATION_OUTER_Y_M,
      LocationMath.DOUBLE_STATION_RED_YAW
  ),
  SINGLE_STATION(
      LedColors.PINK,
      LocationMath.SINGLE_STATION_BLUE_X_M,
      LocationMath.SINGLE_STATION_Y_M - Constants.CRobot.drive.dims.halfBumperWidth_M,
      LocationMath.SINGLE_STATION_ANGLE,
      LocationMath.SINGLE_STATION_RED_X_M,
      LocationMath.SINGLE_STATION_Y_M - Constants.CRobot.drive.dims.halfBumperWidth_M,
      LocationMath.SINGLE_STATION_ANGLE
  );

  public final AlliancePose2d pose;
  public final LedColors stationColor;

  HPIntakeStations(
      LedColors stationColor,
      double blueX,
      double blueY,
      Rotation2d blueYaw,
      double redX,
      double redY,
      Rotation2d redYaw
  ) {
    this.stationColor = stationColor;
    Pose2d bluePose = new Pose2d(blueX, blueY, blueYaw);
    Pose2d redPose = new Pose2d(redX, redY, redYaw);
    pose = new AlliancePose2d(bluePose, redPose);
  }

  HPIntakeStations(LedColors stationColor) {
    pose = null;
    this.stationColor = stationColor;
  }

  public HPIntakeStations getIncr() {
    return HPIntakeStations.values()[((ordinal() + 1) % HPIntakeStations.values().length)];
  }

  public static final class LocationMath {
    /* ========== DOUBLE STATION INNER / OUTER DIMENSIONS ========== */
    public static final double DOUBLE_STATION_COVER_MAX_TO_BORDER_M = Units.inchesToMeters(34.86);
    public static final double DIVIDER_WALL_TO_BORDER_M = Units.inchesToMeters(99.07);
    public static final double DOUBLE_STATION_COVER_MIN_TO_BORDER_M =
        DIVIDER_WALL_TO_BORDER_M - Units.inchesToMeters(34.25);

    /* ========== DERIVED LOADING ZONE DIMENSIONS ========== */
    public static final double LOADING_ZONE_MIN_Y_M =
        FieldDims.THEORETICAL_Y_M - DIVIDER_WALL_TO_BORDER_M;
    public static final double LOADING_ZONE_MAX_Y_M = FieldDims.THEORETICAL_Y_M;

    public static final double LOADING_ZONE_RED_MIN_X_M = Units.inchesToMeters(0.0);
    // (full field / 2) - 61.36Units.inchesToMeters()
    public static final double LOADING_ZONE_RED_MAX_X_M =
        (FieldDims.THEORETICAL_X_M / 2.0) - Units.inchesToMeters(61.36);
    public static final double LOADING_ZONE_BLUE_MIN_X_M =
        FieldDims.THEORETICAL_X_M - LOADING_ZONE_RED_MAX_X_M;
    public static final double LOADING_ZONE_BLUE_MAX_X_M = FieldDims.THEORETICAL_X_M;

    /* ========== DERIVED DOUBLE STATION INNER ROBOT Y ALIGNMENT ========== */
    public static final double DOUBLE_STATION_INNER_MIN_Y_M = Constants.CField.dims.y_M
        - DIVIDER_WALL_TO_BORDER_M + Constants.CRobot.drive.dims.halfBumperWidth_M;
    public static final double DOUBLE_STATION_INNER_MAX_Y_M = Constants.CField.dims.y_M
        - DOUBLE_STATION_COVER_MIN_TO_BORDER_M
        - (Constants.hasClaw ? Constants.CRobot.claw.dims.halfOpenWidth_M : 0.0);
    public static final double DOUBLE_STATION_INNER_Y_M =
        (DOUBLE_STATION_INNER_MIN_Y_M + DOUBLE_STATION_INNER_MAX_Y_M) / 2.0
        + Constants.CField.offsets.shiftDoubleInnerIntakeY_M;

    /* ========== DERIVED DOUBLE STATION OUTER ROBOT Y ALIGNMENT ========== */
    public static final double DOUBLE_STATION_OUTER_MAX_Y_M =
        Constants.CField.dims.y_M - Constants.CRobot.drive.dims.halfBumperWidth_M;
    public static final double DOUBLE_STATION_OUTER_MIN_Y_M = Constants.CField.dims.y_M
        - DOUBLE_STATION_COVER_MAX_TO_BORDER_M
        + (Constants.hasClaw ? Constants.CRobot.claw.dims.halfOpenWidth_M : 0.0);
    public static final double DOUBLE_STATION_OUTER_Y_M =
        (DOUBLE_STATION_OUTER_MAX_Y_M + DOUBLE_STATION_OUTER_MIN_Y_M) / 2.0
        + Constants.CField.offsets.shiftDoubleOuterIntakeY_M;

    /* ========== DERIVED DOUBLE STATION X ALIGNMENT ========== */
    public static final double DOUBLE_STATION_RED_X_M = Units.inchesToMeters(14.0);
    public static final double DOUBLE_STATION_BLUE_X_M =
        Constants.CField.dims.x_M - Units.inchesToMeters(14.0);

    /* ========== DERIVED DOUBLE STATION YAW ALIGNMENT ========== */
    public static final Rotation2d DOUBLE_STATION_BLUE_YAW = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d DOUBLE_STATION_RED_YAW = Rotation2d.fromDegrees(180.0);

    /* ========== SINGLE STATION DIMENSIONS ========== */
    public static final Rotation2d SINGLE_STATION_ANGLE = Rotation2d.fromDegrees(90.0);
    public static final Rotation2d RED_PLATFORM_ANGLE = Rotation2d.fromDegrees(180.0);
    public static final Rotation2d BLUE_PLATFORM_ANGLE = Rotation2d.fromDegrees(0.0);
    public static final double SINGLE_STATION_LENGTH_M = Units.inchesToMeters(22.75);
    public static final double BORDER_TO_SINGLE_STATION_MAX_M = Units.inchesToMeters(103.625);

    /* ========== DERIVED SINGLE STATION X ALIGNMENT ========== */
    public static final double SINGLE_STATION_RED_MIN_X_M =
        BORDER_TO_SINGLE_STATION_MAX_M - SINGLE_STATION_LENGTH_M;
    public static final double SINGLE_STATION_RED_MAX_X_M = BORDER_TO_SINGLE_STATION_MAX_M;
    public static final double SINGLE_STATION_RED_X_M =
        (SINGLE_STATION_RED_MIN_X_M + SINGLE_STATION_RED_MAX_X_M) / 2.0;
    public static final double SINGLE_STATION_BLUE_MIN_X_M =
        Constants.CField.dims.x_M - BORDER_TO_SINGLE_STATION_MAX_M;
    public static final double SINGLE_STATION_BLUE_MAX_X_M =
        SINGLE_STATION_BLUE_MIN_X_M + SINGLE_STATION_LENGTH_M;
    public static final double SINGLE_STATION_BLUE_X_M =
        (SINGLE_STATION_BLUE_MIN_X_M + SINGLE_STATION_BLUE_MAX_X_M) / 2.0;

    /* ========== DERIVED SINGLE STATION Y ALIGNMENT ========== */
    // Final goal (drive into wall for guaranteed alignment)
    public static final double SINGLE_STATION_Y_M = Constants.CField.dims.y_M;
  }
}
