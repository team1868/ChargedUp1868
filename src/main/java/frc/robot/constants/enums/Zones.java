package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

public enum Zones {
  // TODO: give real values for SCORING zone, this is the full field
  SCORING(
      new Translation2d(0.0, 0.0),
      new Translation2d(FieldDims.THEORETICAL_X_M, FieldDims.THEORETICAL_Y_M),
      new Translation2d(0.0, 0.0),
      new Translation2d(FieldDims.THEORETICAL_X_M, FieldDims.THEORETICAL_Y_M)
  ),
  ROBOT_COMMUNITY(
      new Translation2d(
          LocationMath.COMMUNITY_ZONE_BLUE_MIN_X_M + Constants.CRobot.drive.dims.halfBumperLength_M,
          LocationMath.COMMUNITY_ZONE_MIN_Y_M + Constants.CRobot.drive.dims.halfBumperWidth_M
      ),
      new Translation2d(
          LocationMath.COMMUNITY_ZONE_BLUE_MAX_X_M - Constants.CRobot.drive.dims.halfBumperLength_M,
          LocationMath.COMMUNITY_ZONE_MAX_Y_M - Constants.CRobot.drive.dims.halfBumperWidth_M
      ),
      new Translation2d(
          LocationMath.COMMUNITY_ZONE_RED_MIN_X_M + Constants.CRobot.drive.dims.halfBumperLength_M,
          LocationMath.COMMUNITY_ZONE_MIN_Y_M + Constants.CRobot.drive.dims.halfBumperWidth_M
      ),
      new Translation2d(
          LocationMath.COMMUNITY_ZONE_RED_MAX_X_M - Constants.CRobot.drive.dims.halfBumperLength_M,
          LocationMath.COMMUNITY_ZONE_MAX_Y_M - Constants.CRobot.drive.dims.halfBumperWidth_M
      )
  ),
  LOADING(
      new Translation2d(
          HPIntakeStations.LocationMath.LOADING_ZONE_BLUE_MIN_X_M
              + Constants.CRobot.drive.dims.halfBumperLength_M,
          HPIntakeStations.LocationMath.LOADING_ZONE_MIN_Y_M
              + Constants.CRobot.drive.dims.halfBumperWidth_M
      ),
      new Translation2d(
          HPIntakeStations.LocationMath.LOADING_ZONE_BLUE_MAX_X_M
              - Constants.CRobot.drive.dims.halfBumperLength_M,
          HPIntakeStations.LocationMath.LOADING_ZONE_MAX_Y_M
              - Constants.CRobot.drive.dims.halfBumperWidth_M
      ),
      new Translation2d(
          HPIntakeStations.LocationMath.LOADING_ZONE_RED_MIN_X_M
              + Constants.CRobot.drive.dims.halfBumperLength_M,
          HPIntakeStations.LocationMath.LOADING_ZONE_MIN_Y_M
              + Constants.CRobot.drive.dims.halfBumperWidth_M
      ),
      new Translation2d(
          HPIntakeStations.LocationMath.LOADING_ZONE_RED_MAX_X_M
              - Constants.CRobot.drive.dims.halfBumperLength_M,
          HPIntakeStations.LocationMath.LOADING_ZONE_MAX_Y_M
              - Constants.CRobot.drive.dims.halfBumperWidth_M
      )
  ),
  CHARGING_STATION_ROBOT_MAX(
      new Translation2d(
          LocationMath.CHARGING_STATION_BLUE_MIN_X_M
              + Constants.CRobot.drive.dims.halfFrameLength_M,
          LocationMath.CHARGING_STATION_MIN_Y_M + Constants.CRobot.drive.dims.halfFrameWidth_M
      ),
      new Translation2d(
          LocationMath.CHARGING_STATION_BLUE_MAX_X_M
              - Constants.CRobot.drive.dims.halfFrameLength_M,
          LocationMath.CHARGING_STATION_MAX_Y_M - Constants.CRobot.drive.dims.halfFrameWidth_M
      ),
      new Translation2d(
          LocationMath.CHARGING_STATION_RED_MIN_X_M + Constants.CRobot.drive.dims.halfFrameLength_M,
          LocationMath.CHARGING_STATION_MIN_Y_M + Constants.CRobot.drive.dims.halfFrameWidth_M
      ),
      new Translation2d(
          LocationMath.CHARGING_STATION_RED_MAX_X_M - Constants.CRobot.drive.dims.halfFrameLength_M,
          LocationMath.CHARGING_STATION_MAX_Y_M - Constants.CRobot.drive.dims.halfFrameWidth_M
      )
  ),
  CHARGING_STATION_ROBOT_BALANCE(
      new Translation2d(
          LocationMath.CHARGING_STATION_CENTER_BLUE_X_M,
          LocationMath.CHARGING_STATION_MIN_Y_M + Constants.CRobot.drive.dims.halfFrameWidth_M
      ),
      new Translation2d(
          LocationMath.CHARGING_STATION_CENTER_BLUE_X_M,
          LocationMath.CHARGING_STATION_MAX_Y_M - Constants.CRobot.drive.dims.halfFrameWidth_M
      ),
      new Translation2d(
          LocationMath.CHARGING_STATION_CENTER_RED_X_M,
          LocationMath.CHARGING_STATION_MIN_Y_M + Constants.CRobot.drive.dims.halfFrameWidth_M
      ),
      new Translation2d(
          LocationMath.CHARGING_STATION_CENTER_RED_X_M,
          LocationMath.CHARGING_STATION_MAX_Y_M - Constants.CRobot.drive.dims.halfFrameWidth_M
      )
  );

  public final Translation2d blueMin, blueMax, redMin, redMax;

  Zones(Translation2d blueMin, Translation2d blueMax, Translation2d redMin, Translation2d redMax) {
    this.blueMin = blueMin;
    this.blueMax = blueMax;
    this.redMin = redMin;
    this.redMax = redMax;
  }

  public boolean inBlueZone(Translation2d cur) {
    return inZone(cur, false);
  }

  public boolean inRedZone(Translation2d cur) {
    return inZone(cur, true);
  }

  boolean inZone(Translation2d cur, boolean isRed) {
    Translation2d min = isRed ? redMin : blueMin;
    Translation2d max = isRed ? redMax : blueMax;
    return cur.getX() < max.getX() && cur.getY() < max.getY() && cur.getX() > min.getX()
        && cur.getY() > min.getY();
  }

  public static final class LocationMath {
    /* ========== COMMUNITY ZONE DIMENSIONS ========== */
    public static final double COMMUNITY_ZONE_MIN_Y_M = Units.inchesToMeters(0.0);
    public static final double COMMUNITY_ZONE_MAX_Y_M = Units.inchesToMeters(216.03);

    /* ========== DERIVED COMMUNITY ZONE COORDINATES ========== */
    public static final double COMMUNITY_ZONE_BLUE_MIN_X_M = Units.inchesToMeters(0.0);
    public static final double COMMUNITY_ZONE_BLUE_MAX_X_M =
        FieldDims.GRID_HARD_STOPS_TO_COMMUNITY_ZONE + FieldDims.GRID_HARD_STOPS_LENGTH_M;

    public static final double COMMUNITY_ZONE_RED_MIN_X_M =
        FieldDims.THEORETICAL_X_M - COMMUNITY_ZONE_BLUE_MAX_X_M;
    public static final double COMMUNITY_ZONE_RED_MAX_X_M = FieldDims.THEORETICAL_X_M;

    /* ========== DERIVED CHARGING STATION DIMENSIONS ========== */
    public static final double CHARGING_STATION_BLUE_MIN_X_M =
        FieldDims.GRID_HARD_STOPS_LENGTH_M + FieldDims.GRID_HARD_STOPS_TO_CHARGE_STATION_M;
    public static final double CHARGING_STATION_BLUE_MAX_X_M =
        COMMUNITY_ZONE_BLUE_MAX_X_M - FieldDims.TAPE_WIDTH_M;
    public static final double CHARGING_STATION_RED_MIN_X_M =
        COMMUNITY_ZONE_RED_MIN_X_M + FieldDims.TAPE_WIDTH_M;
    public static final double CHARGING_STATION_RED_MAX_X_M =
        FieldDims.THEORETICAL_X_M - CHARGING_STATION_BLUE_MIN_X_M;
    public static final double CHARGING_STATION_CENTER_BLUE_X_M =
        (CHARGING_STATION_BLUE_MIN_X_M + CHARGING_STATION_BLUE_MAX_X_M) / 2;
    public static final double CHARGING_STATION_CENTER_RED_X_M =
        (CHARGING_STATION_RED_MIN_X_M + CHARGING_STATION_RED_MAX_X_M) / 2;

    public static final double BORDER_TO_CHARGING_STATION_M = Units.inchesToMeters(59.39);
    public static final double CHARGING_STATION_TO_DIVIDER_WALL_M = Units.inchesToMeters(59.39);
    public static final double CHARGING_STATION_MIN_Y_M = BORDER_TO_CHARGING_STATION_M;
    public static final double CHARGING_STATION_MAX_Y_M =
        COMMUNITY_ZONE_MAX_Y_M - CHARGING_STATION_TO_DIVIDER_WALL_M;
    public static final double CHARING_STATION_CENTER_Y_M =
        (CHARGING_STATION_MIN_Y_M + CHARGING_STATION_MAX_Y_M) / 2.0;
  }
}
