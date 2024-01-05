package frc.robot.constants.enums;

import edu.wpi.first.math.util.Units;

public enum DrivetrainDimensions {
  COMP_BOT_DIMENSIONS(20.75, 26),
  PRACTICE_BOT_DIMENSIONS(20.75, 26),
  SWERVE_BASE_DIMENSIONS(21.5, 28);

  public final double bumperSegmentDepth_M;
  public double trackLength_M, trackWidth_M, halfTrackLength_M, halfTrackWidth_M;
  public double frameLength_M, frameWidth_M, halfFrameLength_M, halfFrameWidth_M;
  public double bumperLength_M, bumperWidth_M, halfBumperLength_M, halfBumperWidth_M;

  DrivetrainDimensions(double sqrTrackDimensionIn, double sqrFrameDimensionIn) {
    this(sqrTrackDimensionIn, sqrTrackDimensionIn, sqrFrameDimensionIn, sqrFrameDimensionIn, 3.25);
  }

  DrivetrainDimensions(
      double trackLength_IN,
      double trackWidth_IN,
      double frameLength_IN,
      double frameWidth_IN,
      double bumperSegmentDepth_IN
  ) {
    bumperSegmentDepth_M = Units.inchesToMeters(bumperSegmentDepth_IN);
    trackLength_M = Units.inchesToMeters(trackLength_IN);
    trackWidth_M = Units.inchesToMeters(trackWidth_IN);
    halfTrackLength_M = trackLength_M / 2.0;
    halfTrackWidth_M = trackWidth_M / 2.0;

    frameLength_M = Units.inchesToMeters(frameLength_IN);
    frameWidth_M = Units.inchesToMeters(frameWidth_IN);
    halfFrameLength_M = frameLength_M / 2.0;
    halfFrameWidth_M = frameWidth_M / 2.0;
    bumperLength_M = frameLength_M + 2.0 * bumperSegmentDepth_M;
    bumperWidth_M = frameWidth_M + 2.0 * bumperSegmentDepth_M;
    halfBumperLength_M = bumperLength_M / 2.0;
    halfBumperWidth_M = bumperLength_M / 2.0;
  }
}
