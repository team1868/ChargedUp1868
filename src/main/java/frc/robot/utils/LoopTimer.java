package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotAltModes;

public class LoopTimer {
  private static double _loopStart;
  private static double _lastTime;
  private static double _currentTime;
  private static final double CONVERSION_FACTOR = 1000.0;

  public static void markLoopStart() {
    if (RobotAltModes.isLoopTiming) {
      _currentTime = RobotController.getFPGATime() * CONVERSION_FACTOR;
      _loopStart = _currentTime;
    }
  }

  public static void markEvent(String event) {
    if (RobotAltModes.isLoopTiming) {
      _lastTime = _currentTime;
      _currentTime = RobotController.getFPGATime() * CONVERSION_FACTOR;
      System.out.println(event + (_currentTime - _lastTime));
    }
  }

  public static void markCompletion(String event, String system) {
    if (RobotAltModes.isLoopTiming) {
      _lastTime = _currentTime;
      _currentTime = RobotController.getFPGATime() * CONVERSION_FACTOR;
      System.out.println(event + (_currentTime - _lastTime) + system + (_currentTime - _loopStart));
    }
  }

  public static void markCompletion(String system) {
    if (RobotAltModes.isLoopTiming) {
      _currentTime = RobotController.getFPGATime() * CONVERSION_FACTOR;
      System.out.println(system + (_currentTime - _loopStart));
    }
  }
}
