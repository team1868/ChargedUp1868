package frc.robot.constants;

import frc.robot.Ports.LedPorts;
import frc.robot.constants.enums.LedColors;

public enum LedConfs {
  COMP_BOT_CONF(LedPorts.COMP_PORTS),
  PRACTICE_BOT_CONF(LedPorts.PRACTICE_PORTS),
  SWERVE_BASE_CONF(LedPorts.SWERVE_BASE_PORTS),
  TEST_CONF(null);

  public final LedPorts ports;

  LedConfs(LedPorts ports) {
    this.ports = ports;
  }

  public static final int CANDLE_LENGTH = 8;
  public static final int LED_LENGTH = 46; // 38; // 68
  public static final int STRIP_LENGTH = 14;
  public static final int INDICATOR_LENGTH = 10;
  public static final boolean ANIMATION_DIRECTION = false;

  public static final int SUBLEVEL_INCREMENT = 4;
  public static final int INDICATOR_INCREMENT = 3;

  public static final int STRIP_1_START = CANDLE_LENGTH;
  public static final int STRIP_2_START = CANDLE_LENGTH + STRIP_LENGTH + INDICATOR_LENGTH;
  public static final int INDICATOR_START = CANDLE_LENGTH + STRIP_LENGTH;

  public static final int STRIP_HALF_1_START = CANDLE_LENGTH;
  public static final int STRIP_HALF_1_END = STRIP_HALF_1_START + STRIP_LENGTH / 2;

  public static final int STRIP_HALF_2_START = CANDLE_LENGTH + STRIP_LENGTH / 2;
  public static final int STRIP_HALF_2_END = LED_LENGTH;

  public enum ScoringSections {
    BOTTOM(LedColors.PURPLE),
    MIDDLE(LedColors.BLUE),
    TOP(LedColors.GREEN),
    UNKNOWN_SCORING_SECTION(LedColors.OFF);

    public final LedColors sectionColor;
    ScoringSections(LedColors sectionColor) {
      this.sectionColor = sectionColor;
    }
  }

  public enum ScoringSubsections {
    SUB_BOTTOM(LedColors.PINK),
    SUB_MIDDLE(LedColors.YELLOW),
    SUB_TOP(LedColors.RED_ORANGE),
    UNKNOWN_SCORING_SUBSECTION(LedColors.OFF);

    public final LedColors subsectionColor;
    ScoringSubsections(LedColors subsectionColor) {
      this.subsectionColor = subsectionColor;
    }
  }

  public enum LedStates { UNKNOWN_STATE, SCORING_DESIRED, INTAKING_DESIRED }

  public enum LedSections {
    ALL(0, LED_LENGTH),
    CANDLE(0, CANDLE_LENGTH),
    NON_CANDLE(CANDLE_LENGTH, LED_LENGTH),
    INDICATOR(INDICATOR_START, INDICATOR_START + INDICATOR_LENGTH),
    NON_INDICATOR(CANDLE_LENGTH + INDICATOR_LENGTH, LED_LENGTH),
    STRIP(CANDLE_LENGTH, CANDLE_LENGTH + STRIP_LENGTH),
    STRIP_2(STRIP_2_START, STRIP_2_START + STRIP_LENGTH),
    STRIP_THIRD_1(STRIP_1_START, STRIP_1_START + SUBLEVEL_INCREMENT),
    STRIP_THIRD_2(STRIP_1_START + SUBLEVEL_INCREMENT, STRIP_1_START + SUBLEVEL_INCREMENT * 2),
    STRIP_THIRD_3(STRIP_1_START + SUBLEVEL_INCREMENT * 2, STRIP_1_START + STRIP_LENGTH),
    STRIP_2_THIRD_1(STRIP_2_START, STRIP_2_START + SUBLEVEL_INCREMENT),
    STRIP_2_THIRD_2(STRIP_2_START + SUBLEVEL_INCREMENT, STRIP_2_START + SUBLEVEL_INCREMENT * 2),
    STRIP_2_THIRD_3(STRIP_2_START + SUBLEVEL_INCREMENT * 2, STRIP_2_START + STRIP_LENGTH),
    STRIP_HALF_1(STRIP_HALF_1_START, STRIP_HALF_1_END),
    STRIP_HALF_2(STRIP_HALF_2_START, STRIP_HALF_2_END),
    STRIP_2_HALF_1(STRIP_2_START + STRIP_LENGTH / 2, STRIP_2_START + STRIP_LENGTH),
    STRIP_2_HALF_2(STRIP_2_START, STRIP_2_START + STRIP_LENGTH / 2),
    INDICATOR_HALF_1(INDICATOR_START, INDICATOR_START + INDICATOR_LENGTH / 2),
    INDICATOR_HALF_2(INDICATOR_START + INDICATOR_LENGTH / 2, INDICATOR_START + INDICATOR_LENGTH),
    INDICATOR_THIRD_1(INDICATOR_START, INDICATOR_START + INDICATOR_INCREMENT),
    INDICATOR_THIRD_2(
        INDICATOR_START + INDICATOR_INCREMENT, INDICATOR_START + 2 * INDICATOR_INCREMENT
    ),
    INDICATOR_THIRD_3(
        INDICATOR_START + INDICATOR_INCREMENT * 2, INDICATOR_START + INDICATOR_LENGTH
    );

    public final int start;
    public final int end;
    public final int length;

    LedSections(int start, int end) {
      this.start = start;
      this.end = end;
      this.length = end - start;
    }
  }

  public static final int LED_WHITE_LEVEL = 50;

  public static final double ROLL_1_LOWER_DEG = 45;
  public static final double ROLL_1_UPPER_DEG = 135;

  public static final double ROLL_2_LOWER_DEG = 225;
  public static final double ROLL_2_UPPER_DEG = 315;

  public static final double ORIENTATION_BLUE_LOWER_DEG = 135;
  public static final double ORIENTATION_BLUE_UPPER_DEG = 315;
  public static final double ORIENTATION_RED_LOWER_DEG = ORIENTATION_BLUE_UPPER_DEG;
  public static final double ORIENTATION_RED_UPPER_DEG = ORIENTATION_BLUE_LOWER_DEG;

  public static final double LEVEL_THRESHOLD = 2.5;

  // public static final double LEVEL_TIME_S = 5;
  // public static final double RESET_TIME_MS = 0.0001;
}
