package frc.robot.constants.enums;

public enum ScoringLevels {
  UNKNOWN_SCORING_LEVEL(
      "UNKNOWN SCORING LEVEL",
      2,
      1.0,
      0.5,
      ElevatorPositions.ELEVATOR_LEVEL_2_CONE,
      ElevatorPositions.ELEVATOR_LEVEL_2_CUBE
  ),
  // hybrid scoring locations
  SCORING_LEVEL_0(
      "(ZERO) LVL 0",
      0,
      0.5,
      0.6,
      ElevatorPositions.ELEVATOR_LEVEL_0_CONE,
      ElevatorPositions.ELEVATOR_LEVEL_0_CUBE,
      Consts.AUTO_CONE_SHOT_TIMEOUT_S,
      Consts.AUTO_CUBE_SHOT_TIMEOUT_S,
      0.3,
      0.3
  ),
  // middle scoring locations
  SCORING_LEVEL_1(
      "(ONE) LVL 1",
      1,
      0.8,
      0.5,
      ElevatorPositions.ELEVATOR_LEVEL_1_CONE,
      ElevatorPositions.ELEVATOR_LEVEL_1_CUBE
  ),
  // top scoring locations
  SCORING_LEVEL_2(
      "(TWO) LVL 2",
      2,
      1.0,
      0.5,
      ElevatorPositions.ELEVATOR_LEVEL_2_CONE,
      ElevatorPositions.ELEVATOR_LEVEL_2_CUBE
  ),
  SCORING_THROW(
      "THROWING",
      3,
      1.0,
      1.0,
      ElevatorPositions.ELEVATOR_LEVEL_2_CONE,
      ElevatorPositions.ELEVATOR_LEVEL_2_CUBE,
      Consts.AUTO_YEET_TIMEOUT_S,
      Consts.AUTO_YEET_TIMEOUT_S,
      Consts.TELEOP_YEET_TIMEOUT_S,
      Consts.TELEOP_YEET_TIMEOUT_S
  );

  public final String label;
  public final int id;
  public final double conePower;
  public final double cubePower;
  public final ElevatorPositions coneHeight;
  public final ElevatorPositions cubeHeight;

  public final double coneAutoTO;
  public final double coneTeleTO;
  public final double cubeAutoTO;
  public final double cubeTeleTO;

  ScoringLevels(
      String label,
      int id,
      double conePower,
      double cubePower,
      ElevatorPositions coneHeight,
      ElevatorPositions cubeHeight,
      double coneAutoTO,
      double cubeAutoTO,
      double coneTeleTO,
      double cubeTeleTO
  ) {
    this.label = label;
    this.id = id;
    this.conePower = -Math.abs(conePower);
    this.cubePower = cubePower;
    this.coneHeight = coneHeight;
    this.cubeHeight = cubeHeight;
    this.coneAutoTO = coneAutoTO;
    this.coneTeleTO = coneTeleTO;
    this.cubeAutoTO = cubeAutoTO;
    this.cubeTeleTO = cubeTeleTO;
  }

  ScoringLevels(
      String label,
      int id,
      double conePower,
      double cubePower,
      ElevatorPositions coneHeight,
      ElevatorPositions cubeHeight
  ) {
    this(
        label,
        id,
        conePower,
        cubePower,
        coneHeight,
        cubeHeight,
        Consts.AUTO_CONE_SHOT_TIMEOUT_S,
        Consts.AUTO_CUBE_SHOT_TIMEOUT_S,
        Consts.TELEOP_CONE_SHOT_TIMEOUT_S,
        Consts.TELEOP_CUBE_SHOT_TIMEOUT_S
    );
  }

  public double getPower(GamePieceType heldPiece) {
    return heldPiece.isCube() ? cubePower : conePower;
  }

  public double getAutoTO(GamePieceType heldPiece) {
    return heldPiece.isCube() ? cubeAutoTO : coneAutoTO;
  }

  public double getTeleTO(GamePieceType heldPiece) {
    return heldPiece.isCube() ? cubeTeleTO : coneTeleTO;
  }

  public ElevatorPositions getPosition(GamePieceType heldPiece) {
    return heldPiece.isCube() ? cubeHeight : coneHeight;
  }

  // TODO: FIX Ryan made a mistake
  public ScoringLevels getIncr() {
    switch (id) {
      case 0:
        return SCORING_LEVEL_1;
      case 1:
      case 2:
      default:
        return SCORING_LEVEL_2;
    }
  }

  public ScoringLevels getDecr() {
    switch (id) {
      case 2:
        return SCORING_LEVEL_1;
      case 1:
      case 0:
      default:
        return SCORING_LEVEL_0;
    }
  }

  public static final class Consts {
    public static final double CONE_LEVEL_0_POWER = -0.5;
    public static final double CONE_LEVEL_1_POWER = -0.8;
    public static final double CONE_LEVEL_2_POWER = -1.0;
    public static final double CUBE_LEVEL_0_POWER = 0.6;
    public static final double CUBE_LEVEL_1_POWER = 0.5;
    public static final double CUBE_LEVEL_2_POWER = 0.5;
    public static final double CUBE_THROW_POWER = 1.00;

    public static final double TELEOP_CONE_SHOT_TIMEOUT_S = 0.16;
    public static final double TELEOP_CUBE_SHOT_TIMEOUT_S = 0.25;
    public static final double TELEOP_YEET_TIMEOUT_S = 0.3;
    public static final double AUTO_CONE_SHOT_TIMEOUT_S = 0.8;
    public static final double AUTO_CUBE_SHOT_TIMEOUT_S = 0.15;
    public static final double AUTO_YEET_TIMEOUT_S = 0.5;
  }
}
