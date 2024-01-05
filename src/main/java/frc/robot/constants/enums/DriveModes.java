package frc.robot.constants.enums;

/**
 * The Drive function acts like a dispatch to different swerve functionalities based off
 * of mode. Default mode is configurable, but should usually be field relative.
 **/
public enum DriveModes {
  UNKNOWN,
  ROBOT_CENTRIC,
  FIELD_RELATIVE,
  SNAP_TO_ANGLE,
  SNAKE,
  TARGET_RELATIVE,
  CHASE_STATIC_TARGET,
  CHASE_DYNAMIC_TARGET,
  SLEWING_FIELD_RELATIVE,
  FIELD_RELATIVE_SKEW_COMPENSATION,
  FIELD_RELATIVE_ANTI_TIP,
  FIELD_RELATIVE_ROTATION_COMPENSATION_SCALING,
  FIELD_RELATIVE_254
}
