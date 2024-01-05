package frc.robot.constants.enums;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

/*
 * This is a stash spot for various field specific shifts from field derived values
 * It could probably get split into more than 1 file
 */
public enum LocationOffsets {
  THEORETICAL_OFFSETS(28.0, 35.0, 0.0),
  NASA_OFFSETS(
      27.0,
      35.0,
      0.0,
      0.0,
      -6.0,
      FieldDims.NASA_FIELD_DIMS.yDelta_IN,
      0.0,
      0.0,
      new double[] {
          -2.5 - FieldDims.NASA_FIELD_DIMS.yDelta_IN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      2.5,
      new double[] {0.0, -0.5, -2.5, 0.0, 0.0, -2.5, 0.0, -2.5, -2.5}
  ),
  CANADA_OFFSETS(28.0, 35.0, 0.0),
  ARIZONA_OFFSETS(28.0, 35.0, 0.0),
  SVR_OFFSETS(28.0, 35.0, 0.0),
  CURIE_OFFSETS(28.0, 35.0, 0.0),
  CHEZY_OFFSETS(28.0, 35.0, 0.0),
  CALGAMES_OFFSETS(28.0, 35.0, 0.0);

  public final double shiftDoubleInnerIntakeY_M, shiftDoubleOuterIntakeY_M;
  public final double shiftAllBlueScoreX_M, shiftAllRedScoreX_M;
  public final double[] shiftBlueScoreY_M, shiftRedScoreY_M;

  public final double loadingPoseOffset_M;
  // Value when preparing should start
  // TODO update distance
  public final double platformPrepDistance_M;
  // TODO update distance back from cube (basically an offset relative to the cube pose)
  // distance back driven when stowing elevator
  public final double platformIntakeConeOffset_M;

  LocationOffsets(
      double loadingPoseOffset,
      double platformPrepDistance,
      double platformIntakeConeOffset,
      double doubleInnerIntakeY,
      double doubleOuterIntakeY,
      double shiftAllBlueX,
      double shiftAllRedX,
      double shiftAllBlueY,
      double[] shiftBlueY,
      double shiftAllRedY,
      double[] shiftRedY
  ) {
    this.loadingPoseOffset_M = loadingPoseOffset;
    this.platformPrepDistance_M = platformPrepDistance;
    this.platformIntakeConeOffset_M = platformIntakeConeOffset;

    assert (shiftBlueY.length == Constants.NUM_SCORING_LOCATIONS);
    assert (shiftRedY.length == Constants.NUM_SCORING_LOCATIONS);
    shiftBlueScoreY_M = new double[Constants.NUM_SCORING_LOCATIONS];
    shiftRedScoreY_M = new double[Constants.NUM_SCORING_LOCATIONS];

    for (int i = 0; i < Constants.NUM_SCORING_LOCATIONS; i++) {
      shiftBlueScoreY_M[i] = Units.inchesToMeters(shiftAllBlueY + shiftBlueY[i]);
      shiftRedScoreY_M[i] = Units.inchesToMeters(shiftAllRedY + shiftRedY[i]);
    }
    shiftAllBlueScoreX_M = Units.inchesToMeters(shiftAllBlueX);
    shiftAllRedScoreX_M = Units.inchesToMeters(shiftAllRedX);
    shiftDoubleInnerIntakeY_M = Units.inchesToMeters(doubleInnerIntakeY);
    shiftDoubleOuterIntakeY_M = Units.inchesToMeters(doubleOuterIntakeY);
  }

  LocationOffsets(LocationOffsets copy) {
    this.loadingPoseOffset_M = copy.loadingPoseOffset_M;
    this.platformPrepDistance_M = copy.platformPrepDistance_M;
    this.platformIntakeConeOffset_M = copy.platformIntakeConeOffset_M;
    this.shiftDoubleInnerIntakeY_M = copy.shiftDoubleInnerIntakeY_M;
    this.shiftDoubleOuterIntakeY_M = copy.shiftDoubleOuterIntakeY_M;
    this.shiftAllBlueScoreX_M = copy.shiftAllBlueScoreX_M;
    this.shiftAllRedScoreX_M = copy.shiftAllRedScoreX_M;
    this.shiftBlueScoreY_M = copy.shiftBlueScoreY_M;
    this.shiftRedScoreY_M = copy.shiftRedScoreY_M;
  }
  LocationOffsets(
      double loadingPoseOffset, double platformPrepDistance, double platformIntakeConeOffset
  ) {
    this.loadingPoseOffset_M = loadingPoseOffset;
    this.platformPrepDistance_M = platformPrepDistance;
    this.platformIntakeConeOffset_M = platformIntakeConeOffset;

    shiftDoubleInnerIntakeY_M = 0.0;
    shiftDoubleOuterIntakeY_M = 0.0;
    shiftAllBlueScoreX_M = 0.0;
    shiftAllRedScoreX_M = 0.0;
    shiftBlueScoreY_M = new double[Constants.NUM_SCORING_LOCATIONS];
    shiftRedScoreY_M = new double[Constants.NUM_SCORING_LOCATIONS];
  }

  public final double getBlueScoringY(int id) {
    return FieldDims.THEORETICAL_SCORING_POSITION_Y_M[id] + shiftBlueScoreY_M[id];
  }
  public final double getRedScoringY(int id) {
    return FieldDims.THEORETICAL_SCORING_POSITION_Y_M[id] + shiftRedScoreY_M[id];
  }
}
