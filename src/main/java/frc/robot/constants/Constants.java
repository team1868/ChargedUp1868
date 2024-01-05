package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.enums.FieldVersions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.HPIntakeStations;
import frc.robot.constants.enums.ModuleModels;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.constants.enums.ScoringLocations;

public class Constants {
  public static final int NUM_SCORING_LOCATIONS = 9;

  public static final RobotVersions CRobot = RobotVersions.CompBot;
  public static final FieldVersions CField = FieldVersions.THEORETICAL_FIELD;

  // if we do our configurations right, this shouldn't be necessary
  public static final boolean isCompBot = CRobot == RobotVersions.CompBot;
  public static final boolean isPracticeBot = CRobot == RobotVersions.PracticeBot;
  public static final boolean isSwerveBase = CRobot == RobotVersions.SwerveBase;

  public static final boolean isNASAField = CField == FieldVersions.NASA_FIELD;
  public static final boolean isCurie = CField == FieldVersions.CURIE_FIELD;
  public static final boolean isSVR = CField == FieldVersions.SVR_FIELD;
  public static final boolean isArizona = CField == FieldVersions.ARIZONA_FIELD;
  public static final boolean isCanada = CField == FieldVersions.CANADA_FIELD;
  public static final boolean isSDSMK4 = CRobot.drive.type == ModuleModels.ModuleTypes.SDS_MK4;
  public static final boolean isSDSMK4I = CRobot.drive.type == ModuleModels.ModuleTypes.SDS_MK4I;
  public static final boolean isCANEncoder = CRobot == RobotVersions.SwerveBase;

  // These are high level robot configurations that fundamentally change robot behavior and
  // control schemes, this also allows for manual overrides. I don't like this, would love to see
  // alternatives... These should really become part of the top level RobotVersions
  public static final boolean hasClaw = CRobot.claw != null;
  public static final boolean isKlaw =
      CRobot.claw == ClawConfs.COMP_KLAW || CRobot.claw == ClawConfs.PRACTICE_KLAW;
  public static final boolean isPneumaticClaw =
      isKlaw; // || CRobot.claw == ClawConfs.PNEUMATIC_CLAW;
  public static final boolean hasLed = CRobot.led != null;
  public static final boolean hasElevator = CRobot.elevator != null;
  public static final boolean hasVision = CRobot.vision != null;

  /// need to move
  // consts
  /* ========== ROBOT OFFSET DIMENSIONS ========== */
  // TODO tune and test this value (location of robot when claw is against back of
  // loading station
  // when extended)

  public static final double LOADING_POSE_OFFSET_M = Units.inchesToMeters(27);
  // Value when preparing should start
  public static final double PLATFORM_PREP_DIST_M =
      Units.inchesToMeters(35); // TODO update distance
  public static final double PLATFORM_INTAKE_CONE_OFFSET_M =
      Units.inchesToMeters(0); // TODO update distance back
                               // from cube (basically an
                               // offset relative to the
                               // cube
                               // pose)
  // distance back driven when stowing elevator

  // TODO update distance
  public static final double PLATFORM_BACKOUT_DIST_M = Units.feetToMeters(6);

  public static final double LOOP_PERIOD_MS = 20.0;
  public static final double LOOP_PERIOD_S = Units.millisecondsToSeconds(LOOP_PERIOD_MS);

  // defaults
  public static final ScoringLevels DEFAULT_SCORING_LEVEL = ScoringLevels.SCORING_LEVEL_2;
  public static final ScoringLocations DEFAULT_SCORING_LOCATION =
      ScoringLocations.SCORING_LOCATION_9;
  public static final HPIntakeStations DEFAULT_INTAKE_STATION =
      HPIntakeStations.DOUBLE_STATION_INNER;
  public static final GamePieceType DEFAULT_GAME_PIECE = GamePieceType.CONE_GAME_PIECE;

  public static final class CTREConstants {
    /* ============= Falcon Constants2 ============= */
    // ticks per motor rotation
    public static final double FALCON_ENCODER_TICKS = 2048.0;
    // TODO: measure this free speed on blocks
    // 6380 +/- 10%
    public static final double MAX_FALCON_RPM = 5800.0;

    public static final double CANCODER_ENCODER_TICKS = 4096.0;

    // multiply to convert Constants2
    public static final double FALCON_TO_RPS = 10.0 / FALCON_ENCODER_TICKS;
    public static final double FALCON_TO_RPM = 60.0 * FALCON_TO_RPS;
    public static final double CANCODER_TO_RPS = 10.0 / CANCODER_ENCODER_TICKS;
    public static final double CANCODER_TO_RPM = 60.0 * CANCODER_TO_RPS;
    public static final double RPS_TO_FALCON = 1.0 / FALCON_TO_RPS;
    public static final double RPM_TO_FALCON = 1.0 / FALCON_TO_RPM;
    public static final double RPS_TO_CANCODER = 1.0 / CANCODER_TO_RPS;
    public static final double RPM_TO_CANCODER = 1.0 / CANCODER_TO_RPM;
    public static final double FALCON_TO_DEG = 360.0 / FALCON_ENCODER_TICKS;
    public static final double DEG_TO_FALCON = FALCON_ENCODER_TICKS / 360.0;
    public static final double DEG_TO_CANCODER = CANCODER_ENCODER_TICKS / 360.0;
    public static final double FALCON_TO_RAD = FALCON_TO_DEG * Math.PI / 180.0;
    public static final double RAD_TO_FALCON = 180.0 / (FALCON_TO_DEG * Math.PI);
    public static final double FALCON_TICKS_TO_ROT = 1.0 / FALCON_ENCODER_TICKS;
    public static final double ROT_TO_FALCON_TICKS = FALCON_ENCODER_TICKS;
  }

  public static final class Sensors {
    public static final boolean INVERT_GYRO = false;

    public static final Rotation2d GYRO_ZERO_BLUE = Rotation2d.fromDegrees(0);
    public static final Rotation2d GYRO_ZERO_RED = GYRO_ZERO_BLUE.plus(Rotation2d.fromDegrees(180));

    public static final double POV_ZERO_BLUE_DEG = 0;
    public static final double POV_ZERO_RED_DEG = POV_ZERO_BLUE_DEG + 180;
  }

  /* For Path Planner */
  // TODO tune for trapezoidal control
  public static final double AUTO_MAX_SPEED_MPS = 3.4;
  public static final double AUTO_XY_TRANSLATION_MAX_A_MPS_SQ = 1.8;
  public static final double CHARGE_MAX_SPEED_MPS = 2.0;
  public static final double CHARGE_XY_TRANSLATION_MAX_A_MPS_SQ = 2.0;
  public static final double CHARGE_TRAVERSE_MAX_SPEED_MPS = 1.4;
  public static final double CHARGE_TRAVERSE_XY_TRANSLATION_MAX_A_MPS_SQ = 1.6;

  /* For Drivetrain Auto */
  public static final double THETA_AUTO_SCORE_TOLERANCE_DEG = 45;

  public static final double FORCED_WHEEL_ALIGNMENT_SPEED_MPS = 1;

  public static final double TRANSLATION_MAX_TRIM_SPEED_MPS = 1;
  public static final Rotation2d ANGLE_MAX_TRIM_SPEED_DPS = Rotation2d.fromDegrees(90.0);

  public static final double SWERVE_SKEW_COMPENSATION_COEFFICIENT = 0.0;

  /* ========== SCORING PREALIGN OFFSET VALUES ========== */
  public static final double SCORING_PREALIGN_RED_OFFSET_X_M = Units.inchesToMeters(-6); // -12
  public static final double SCORING_PREALIGN_RED_OFFSET_Y_M = Units.inchesToMeters(0);
  public static final double SCORING_PREALIGN_RED_OFFSET_THETA_DEG = 0; // degree

  /* ========== SCORING PREALIGN OFFSET POSES ========== */
  public static final Translation2d SCORING_PREALIGN_RED_OFFSET_XY =
      new Translation2d(SCORING_PREALIGN_RED_OFFSET_X_M, SCORING_PREALIGN_RED_OFFSET_Y_M);
  public static final Transform2d SCORING_PREALIGN_RED_OFFSET = new Transform2d(
      SCORING_PREALIGN_RED_OFFSET_XY, Rotation2d.fromDegrees(SCORING_PREALIGN_RED_OFFSET_THETA_DEG)
  );

  /* ========== POST SCORING OFFSET VALUES ========== */
  public static final double POST_SCORING_RED_OFFSET_X_M = Units.inchesToMeters(-12);
  public static final double POST_SCORING_RED_OFFSET_Y_M = Units.inchesToMeters(0);
  public static final double POST_SCORING_RED_OFFSET_THETA_DEG = 0; // degree

  /* ========== POST SCORING OFFSET POSES ========== */
  public static final Translation2d POST_SCORING_RED_OFFSET_XY =
      new Translation2d(POST_SCORING_RED_OFFSET_X_M, POST_SCORING_RED_OFFSET_Y_M);
  public static final Transform2d POST_SCORING_RED_OFFSET = new Transform2d(
      POST_SCORING_RED_OFFSET_XY, Rotation2d.fromDegrees(POST_SCORING_RED_OFFSET_THETA_DEG)
  );

  // Timeout
  public static final double ALIGN_SHOT_TIMEOUT_S = 2; // second

  public static final class Control {
    public static final double STICK_DEADBAND = 0.1; // TODO tune, too low
    public static final double STICK_NET_DEADBAND = 0.125; // TODO tune, too low

    public static final boolean IS_OPEN_LOOP = false; // swerve
  }
}
