package frc.robot.constants;

import frc.robot.Robot;

public final class RobotAltModes {
  /* --- Test Mode Configurations --- */
  public static final boolean isTestMode = false;
  public static final double TEST_MODE_COEFFICIENT = 0.5;

  /* --- Debug Modes */
  public static final boolean isVerboseMode = false;
  public static final boolean isLoopTiming = false;
  public static final boolean isAutoTuning = false;
  public static final boolean isPoseTuning = false;
  public static final boolean isPIDTuningMode = false;
  public static final boolean isUnprofiledPIDMode = false;
  public static final boolean isElevatorTuningMode = false;

  /* --- Sim Actionability --- */
  public static final boolean isSim = Robot.isSimulation();
  public static final boolean isSimElevator = false;

  // TODO: Move to current robot configurations
  public static final boolean isVisionMode = false;
}
