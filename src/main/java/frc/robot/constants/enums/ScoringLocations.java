package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;
import frc.robot.constants.LedConfs.ScoringSections;
import frc.robot.constants.LedConfs.ScoringSubsections;
import frc.robot.utils.AlliancePose2d;

public enum ScoringLocations {
  // Unknown location will default to locaation 9 behavior
  UNKNOWN_SCORING_LOCATION(
      8,
      ScoringSections.UNKNOWN_SCORING_SECTION,
      ScoringSubsections.UNKNOWN_SCORING_SUBSECTION,
      CameraSets.CAMERA_0,
      CameraSets.CAMERA_1
  ),
  // Cone, wire cover section, adjacent to the wall
  SCORING_LOCATION_1(
      0,
      ScoringSections.BOTTOM,
      ScoringSubsections.SUB_BOTTOM,
      CameraSets.CAMERA_1,
      CameraSets.CAMERA_0
  ),
  // Cube, wire cover section, cube platform
  SCORING_LOCATION_2(1, ScoringSections.BOTTOM, ScoringSubsections.SUB_MIDDLE, CameraSets.CAMERA_0),
  // Cone, wire cover section, middle most
  SCORING_LOCATION_3(
      2,
      ScoringSections.BOTTOM,
      ScoringSubsections.SUB_TOP,
      CameraSets.CAMERA_0,
      CameraSets.CAMERA_1
  ),
  // Cone, coopertition section, closest to wire cover
  SCORING_LOCATION_4(
      3,
      ScoringSections.MIDDLE,
      ScoringSubsections.SUB_BOTTOM,
      CameraSets.CAMERA_1,
      CameraSets.CAMERA_0
  ),
  // Cube, coopertition section, cube platform
  SCORING_LOCATION_5(4, ScoringSections.MIDDLE, ScoringSubsections.SUB_MIDDLE, CameraSets.CAMERA_0),
  // Cone, coopertition section, closest to loading zone
  SCORING_LOCATION_6(
      5,
      ScoringSections.MIDDLE,
      ScoringSubsections.SUB_TOP,
      CameraSets.CAMERA_0,
      CameraSets.CAMERA_1
  ),
  // Cone, loading zone section, middle most
  SCORING_LOCATION_7(
      6,
      ScoringSections.TOP,
      ScoringSubsections.SUB_BOTTOM,
      CameraSets.CAMERA_1,
      CameraSets.CAMERA_0
  ),
  // Cube, loading zone section, cube platform
  SCORING_LOCATION_8(7, ScoringSections.TOP, ScoringSubsections.SUB_MIDDLE, CameraSets.CAMERA_0),
  // Cone, loading zone section, adjacent to laoding zone
  SCORING_LOCATION_9(
      8, ScoringSections.TOP, ScoringSubsections.SUB_TOP, CameraSets.CAMERA_0, CameraSets.CAMERA_1
  );

  /* ========== SCORING ANGLE ========== */
  public static final Rotation2d RED_SCORING_ANGLE = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d BLUE_SCORING_ANGLE = Rotation2d.fromDegrees(180.0);

  public final int index;
  public final AlliancePose2d pose;
  public final ScoringSections section;
  public final ScoringSubsections subsection;
  public final CameraSets redCameras, blueCameras;

  ScoringLocations(
      int index, ScoringSections section, ScoringSubsections subsection, CameraSets camera
  ) {
    this(index, section, subsection, camera, camera);
  }

  ScoringLocations(
      int index,
      AlliancePose2d poses,
      ScoringSections section,
      ScoringSubsections subsection,
      CameraSets camera
  ) {
    this(index, poses, section, subsection, camera, camera);
  }

  ScoringLocations(
      int index,
      Pose2d bluePose,
      Pose2d redPose,
      ScoringSections section,
      ScoringSubsections subsection,
      CameraSets camera
  ) {
    this(index, bluePose, redPose, section, subsection, camera, camera);
  }

  ScoringLocations(
      int index,
      Pose2d bluePose,
      Pose2d redPose,
      ScoringSections section,
      ScoringSubsections subsection,
      CameraSets blueCameras,
      CameraSets redCameras
  ) {
    this(
        index, new AlliancePose2d(bluePose, redPose), section, subsection, blueCameras, redCameras
    );
  }

  ScoringLocations(
      int index,
      AlliancePose2d poses,
      ScoringSections section,
      ScoringSubsections subsection,
      CameraSets blueCameras,
      CameraSets redCameras
  ) {
    this.index = index;
    this.section = section;
    this.subsection = subsection;
    pose = poses;
    this.blueCameras = blueCameras;
    this.redCameras = redCameras;
  }

  ScoringLocations(
      int index,
      ScoringSections section,
      ScoringSubsections subsection,
      CameraSets blueCameras,
      CameraSets redCameras
  ) {
    this.index = index;
    this.section = section;
    this.subsection = subsection;
    pose = derivedPose();
    this.blueCameras = blueCameras;
    this.redCameras = redCameras;
  }

  private AlliancePose2d derivedPose() {
    /* ========== DERIVED SCORING X COORDINATES ========== */
    double blueX = FieldDims.GRID_HARD_STOPS_LENGTH_M
        + Constants.CRobot.drive.dims.halfBumperLength_M
        + Constants.CField.offsets.shiftAllBlueScoreX_M;
    double redX = (Constants.CField.dims.x_M - FieldDims.GRID_HARD_STOPS_LENGTH_M)
        - Constants.CRobot.drive.dims.halfBumperLength_M
        + Constants.CField.offsets.shiftAllRedScoreX_M;

    Pose2d bluePose =
        new Pose2d(blueX, Constants.CField.offsets.getBlueScoringY(index), BLUE_SCORING_ANGLE);
    Pose2d redPose =
        new Pose2d(redX, Constants.CField.offsets.getRedScoringY(index), RED_SCORING_ANGLE);
    return new AlliancePose2d(bluePose, redPose);
  }

  // TODO FIX RYAN'S DUMB
  public ScoringLocations getIncr() {
    switch (index) {
      case 0:
        return SCORING_LOCATION_2;
      case 1:
        return SCORING_LOCATION_3;
      case 2:
        return SCORING_LOCATION_4;
      case 3:
        return SCORING_LOCATION_5;
      case 4:
        return SCORING_LOCATION_6;
      case 5:
        return SCORING_LOCATION_7;
      case 6:
        return SCORING_LOCATION_8;
      case 7:
        return SCORING_LOCATION_9;
      case 8:
        return SCORING_LOCATION_1;
      default:
        return SCORING_LOCATION_9;
    }
  }

  public ScoringLocations getDecr() {
    switch (index) {
      case 0:
        return SCORING_LOCATION_9;
      case 1:
        return SCORING_LOCATION_1;
      case 2:
        return SCORING_LOCATION_2;
      case 3:
        return SCORING_LOCATION_3;
      case 4:
        return SCORING_LOCATION_4;
      case 5:
        return SCORING_LOCATION_5;
      case 6:
        return SCORING_LOCATION_6;
      case 7:
        return SCORING_LOCATION_7;
      case 8:
        return SCORING_LOCATION_8;
      default:
        return SCORING_LOCATION_8;
    }
  }

  public CameraSets getScoringCamera(boolean isRed) {
    return isRed ? redCameras : blueCameras;
  }

  public static Rotation2d getScoreAngle(Alliance alliance) {
    return alliance == Alliance.Red ? RED_SCORING_ANGLE : BLUE_SCORING_ANGLE;
  }
}
