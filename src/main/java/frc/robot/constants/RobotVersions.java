package frc.robot.constants;

import frc.robot.Ports.PneumaticPorts;

public enum RobotVersions {
  CompBot(
      DrivetrainConfs.COMP_BOT_CONFS,
      ClawConfs.COMP_KLAW,
      ElevatorConfs.COMP_BOT_CONF,
      VisionConfs.COMP_BOT_CONF,
      LedConfs.COMP_BOT_CONF,
      PneumaticPorts.COMP_PORTS
  ),
  PracticeBot(
      DrivetrainConfs.PRACTICE_BOT_CONFS,
      null,
      null,
      // ClawConfs.PRACTICE_KLAW,
      // ElevatorConfs.PRACTICE_BOT_CONF,
      VisionConfs.PRACTICE_BOT_CONF,
      // LedConfs.PRACTICE_BOT_CONF,
      null,
      PneumaticPorts.PRACTICE_PORTS
  ),
  SwerveBase(
      DrivetrainConfs.SWERVE_BASE_CONFS, null, null, VisionConfs.SWERVE_BASE_CONF, null, null
  ),
  TestBoard(null, null, null, null, null, null);

  public final DrivetrainConfs drive;
  public final ClawConfs claw;
  public final ElevatorConfs elevator;
  public final VisionConfs vision;
  public final LedConfs led;
  public final PneumaticPorts air;

  RobotVersions(
      DrivetrainConfs drive,
      ClawConfs claw,
      ElevatorConfs elevator,
      VisionConfs vision,
      LedConfs led,
      PneumaticPorts air
  ) {
    this.drive = drive;
    this.claw = claw;
    this.elevator = elevator;
    this.vision = vision;
    this.led = led;
    this.air = air;
  }
}
