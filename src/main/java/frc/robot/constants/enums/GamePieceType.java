package frc.robot.constants.enums;

public enum GamePieceType {
  UNKNOWN_GAME_PIECE("UNKNOWN/NONE", LedColors.OFF),
  CUBE_GAME_PIECE(LedColors.CUBE),
  CONE_GAME_PIECE(LedColors.CONE);

  public final String label;
  public final LedColors intakeColor;

  GamePieceType(LedColors intakeColor) {
    label = this.name().replace("_GAME_PIECE", "");
    this.intakeColor = intakeColor;
  }

  GamePieceType(String label, LedColors intakeColor) {
    this.label = label;
    this.intakeColor = intakeColor;
  }

  public boolean isCube() {
    return this == CUBE_GAME_PIECE;
  }

  public boolean isUnknown() {
    return this == UNKNOWN_GAME_PIECE;
  }
}
