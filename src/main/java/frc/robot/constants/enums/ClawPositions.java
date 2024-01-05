package frc.robot.constants.enums;

public enum ClawPositions {
  CLAW_POSITION_UNKNOWN(0),
  CLAW_OPEN_POS_TICKS(2048),
  CLAW_MAX_TICKS(2048),
  CLAW_CLOSED_POS_TICKS(0),
  CLAW_MIN_TICKS(0),
  CLAW_OPEN_POS_THRESHOLD(2028),
  CLAW_CLOSED_POS_THRESHOLD(21);

  public final int ticks;

  ClawPositions(int ticks) {
    this.ticks = ticks;
  }

  private static final int CLAW_MIN_FALCON_TICKS = 0;
  private static final int CLAW_MAX_FALCON_TICKS = 2048;
  private static final int CLAW_OPEN_TICKS = 1024;

  public static final int CLAW_TOLERANCE_TICKS = 1000;
}
