package frc.robot.constants.enums;

import frc.robot.constants.Constants;

// TODO: redo this somehow... must be derived from both current robot and current field somehow...
public enum ElevatorPositions {
  // in falcon ticks
  ELEVATOR_UNKNOWN_POSITION(0),
  ELEVATOR_MIN_TICKS(1, 1),
  ELEVATOR_STOW_TICKS(1000, 2000), // stow location, above 0 by ~ 1x or 2x tolerance
  ELEVATOR_GROUND_CUBE_INTAKE(1250, 2000), // Height for cubes from ground intake
  ELEVATOR_GROUND_CONE_INTAKE(4300, 5700),
  ELEVATOR_LEVEL_0_CONE(1000, 14000), // scoring height for cone on level 0
  ELEVATOR_LEVEL_0_CUBE(1000, 16500), // scoring height for cube on level 0
  ELEVATOR_LEVEL_2_CONE_OPTIMIZATION(12000, ELEVATOR_UNKNOWN_POSITION.ticks),
  ELEVATOR_LEVEL_2_CONE_MANUAL_OPTIMIZATION(15000, ELEVATOR_UNKNOWN_POSITION.ticks),
  ELEVATOR_TIP_PREVENTION_THRESHOLD(30000, 12000), // Height that will engage anti tip
  ELEVATOR_DROP_CONE_INTAKE(7000),
  ELEVATOR_LEVEL_1_CONE(46400, 52000), // scoring height for cone on level 1
  ELEVATOR_LEVEL_1_CUBE(43000, 47000), // scoring height for cube on level 1
  AUTO_ELEVATOR_LEVEL_1_CUBE(26400, 23400), // 45001, // 23400, // 24400, // 26400,
  AUTO_ELEVATOR_LEVEL_2_CUBE(43000, 44500), // 44501, // 43001,
  ELEVATOR_PLATFORM_CONE_INTAKE(66300, 57000), // Height for intaking from the platform
  ELEVATOR_PLATFORM_CUBE_INTAKE(62500, 63000), // Height for intaking from the platform
  ELEVATOR_LEVEL_2_CUBE(64500, 69500), // scoring height for cube on level 2
  ELEVATOR_LEVEL_2_CONE(67500, 69500), // scoring height for cone on level 2
  ELEVATOR_MAX_TICKS(68460, 71000);

  public final int ticks;

  // probably add a 3'rd option that can be used for field specific things
  ElevatorPositions(int compTicks, int pracTick) {
    ticks = Constants.isCompBot   ? compTicks
        : Constants.isPracticeBot ? pracTick
                                  : ELEVATOR_MIN_FALCON_TICKS;
  }

  ElevatorPositions(int everyRobot) {
    ticks = everyRobot;
  }

  public static final int ELEVATOR_MIN_FALCON_TICKS = 1;
  public static final int ELEVATOR_MAX_FALCON_TICKS = Constants.isCompBot ? 67800 : 71000;

  public boolean isUnknown() {
    return this == ELEVATOR_UNKNOWN_POSITION;
  }
}
