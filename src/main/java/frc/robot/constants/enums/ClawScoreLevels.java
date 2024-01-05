package frc.robot.constants.enums;

public enum ClawScoreLevels {
  CLAW_SCORE_UNKNOWN(GamePieceType.CONE_GAME_PIECE, ScoringLevels.SCORING_LEVEL_2),
  CONE_LEVEL_0(GamePieceType.CONE_GAME_PIECE, ScoringLevels.SCORING_LEVEL_0),
  CONE_LEVEL_1(GamePieceType.CONE_GAME_PIECE, ScoringLevels.SCORING_LEVEL_1),
  CONE_LEVEL_2(GamePieceType.CONE_GAME_PIECE, ScoringLevels.SCORING_LEVEL_2),
  CUBE_LEVEL_0(GamePieceType.CUBE_GAME_PIECE, ScoringLevels.SCORING_LEVEL_0),
  CUBE_LEVEL_1(GamePieceType.CUBE_GAME_PIECE, ScoringLevels.SCORING_LEVEL_1),
  CUBE_LEVEL_2(GamePieceType.CUBE_GAME_PIECE, ScoringLevels.SCORING_LEVEL_2);

  public final double power;
  public final double autoTimeout;
  public final double teleTimeout;

  ClawScoreLevels(double power, double autoTO, double teleTO) {
    this.power = power;
    this.autoTimeout = autoTO;
    this.teleTimeout = teleTO;
  }

  // explicitly tie it to the SoringLevels enum
  ClawScoreLevels(GamePieceType piece, ScoringLevels level) {
    this.power = level.getPower(piece);
    this.autoTimeout = level.getAutoTO(piece);
    this.teleTimeout = level.getTeleTO(piece);
  }
}
