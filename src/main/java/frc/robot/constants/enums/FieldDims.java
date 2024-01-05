package frc.robot.constants.enums;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.utils.UnitUtils;

public enum FieldDims {
  THEORETICAL_FIELD_DIMS(),
  NASA_FIELD_DIMS(FieldDims.THEORETICAL_X_IN - 72.25, FieldDims.THEORETICAL_Y_IN - 30.5),
  CANADA_FIELD_DIMS(),
  ARIZONA_FIELD_DIMS(),
  SVR_FIELD_DIMS(),
  CURIE_FIELD_DIMS(),
  CHEZY_FIELD_DIMS(),
  CALGAMES_FIELD_DIMS();

  // Base Values
  public final double x_IN, x_M;
  public final double y_IN, y_M;
  public final double halfX_IN, halfX_M;
  public final double halfY_IN, halfY_M;
  public final double xDelta_IN, xDelta_M;
  public final double yDelta_IN, yDelta_M;

  FieldDims(double x, double y) {
    x_IN = x;
    x_M = Units.inchesToMeters(x);
    y_IN = y;
    y_M = Units.inchesToMeters(y);

    halfX_IN = x / 2.0;
    halfX_M = Units.inchesToMeters(halfX_IN);
    halfY_IN = y / 2.0;
    halfY_M = Units.inchesToMeters(halfY_IN);

    xDelta_IN = x_IN - THEORETICAL_X_IN;
    xDelta_M = Units.inchesToMeters(xDelta_IN);
    yDelta_IN = y_IN - THEORETICAL_Y_IN;
    yDelta_M = Units.inchesToMeters(yDelta_IN);
  }
  FieldDims() {
    this(THEORETICAL_X_IN, THEORETICAL_Y_IN);
  }

  // CONSTANT VALUES THAT SHOULD NEVER CHANGE
  public static final double THEORETICAL_X_IN = 651.25,
                             THEORETICAL_X_M = Units.inchesToMeters(THEORETICAL_X_IN);
  public static final double THEORETICAL_Y_IN = 315.5,
                             THEORETICAL_Y_M = Units.inchesToMeters(THEORETICAL_Y_IN);
  public static final double TAPE_WIDTH_IN = 2.0,
                             TAPE_WIDTH_M = Units.inchesToMeters(TAPE_WIDTH_IN);

  public static final double GRID_HARD_STOPS_TO_STAGING_MARK_M = Units.inchesToMeters(224.0);
  // Wall to scoring hard stop dimension (the construction is 54.05 and the manual includes tape)
  public static final double GRID_HARD_STOPS_LENGTH_M = Units.inchesToMeters(56.25) - TAPE_WIDTH_IN;
  public static final double GRID_HARD_STOPS_TO_CHARGE_STATION_M = Units.inchesToMeters(60.69);

  /* ========== THEORETICAL SCORING Y ALIGNMENT ========== */
  public static final double[] THEORETICAL_SCORING_POSITION_Y_M =
      UnitUtils.inchesToMeters(new double[] {
          20.0, 42.0, 64.0, 86.0, 108.0, 130.0, 152.0, 174.0, 196.0});

  public static final double STAGING_MARK_TO_COMMUNITY_ZONE_M = Units.inchesToMeters(85.13);

  public static final double GRID_HARD_STOPS_TO_COMMUNITY_ZONE =
      GRID_HARD_STOPS_TO_STAGING_MARK_M - STAGING_MARK_TO_COMMUNITY_ZONE_M;

  public static final double BLUE_SCORING_X_M =
      GRID_HARD_STOPS_LENGTH_M + Constants.CRobot.drive.dims.halfBumperLength_M;
}
