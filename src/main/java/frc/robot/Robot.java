// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.utils.LoopTimer;

public class Robot extends TimedRobot {
  private Command _autonomousCommand;

  private final RobotContainer _robotContainer;
  private final CommandScheduler _scheduler;

  public Robot() {
    super(Constants.LOOP_PERIOD_S);

    LoopTimer.markLoopStart();

    _scheduler = CommandScheduler.getInstance();
    _robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    _robotContainer.configFMSData();
    _robotContainer.robotInit();
  }

  @Override
  public void robotPeriodic() {
    _scheduler.run();
    _robotContainer.periodic();
    if (RobotAltModes.isLoopTiming) {
      System.out.println();
    }
  }

  @Override
  public void disabledInit() {
    _robotContainer.onDisable();
    ;
  }

  @Override
  public void disabledPeriodic() {
    _robotContainer.disabledPeriodic();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    _robotContainer.configFMSData();
    _robotContainer.autonomousInit();
    _autonomousCommand = _robotContainer._curAutoSelected.command;

    if (_autonomousCommand != null) {
      _autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    _robotContainer.log();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    _robotContainer.configFMSData();
    _robotContainer.teleopInit();
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();
      _autonomousCommand = null;
    }
  }

  @Override
  public void teleopPeriodic() {
    _robotContainer.log();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public void simulationInit() {
    _robotContainer.simulationInit();
  }

  public void simulationPeriodic() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
