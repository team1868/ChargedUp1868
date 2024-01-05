package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants.Control;

public class InputUtils {
  public static double scaleJoystickX_MPS(double rawXAxis, double xyNet, double maxSpeed_mps) {
    // double scaledXY = xyNet < Control.STICK_NET_DEADBAND ? 0 : xyNet;
    // double scaledXY =
    //     xyNet < Control.STICK_NET_DEADBAND ? 0 : Math.signum(rawXAxis) * Math.pow(xyNet, 2);
    double scaledXY = xyNet < Control.STICK_NET_DEADBAND ? 0 : Math.pow(xyNet, 3);
    double scaledX = rawXAxis * scaledXY / xyNet;
    return scaledX * maxSpeed_mps;
  }

  public static double scaleJoystickY_MPS(double rawYAxis, double xyNet, double maxSpeed_mps) {
    // double scaledXY = xyNet < Control.STICK_NET_DEADBAND ? 0 : xyNet;
    // double scaledXY =
    //     xyNet < Control.STICK_NET_DEADBAND ? 0 : Math.signum(rawYAxis) * Math.pow(xyNet, 2);
    double scaledXY = Math.abs(xyNet) < Control.STICK_NET_DEADBAND ? 0 : Math.pow(xyNet, 3);
    double scaledY = rawYAxis * scaledXY / xyNet;
    return scaledY * maxSpeed_mps;
  }

  public static double ScaleJoystickTheta_RADPS(double rawAxis, Rotation2d maxAngularSpeed) {
    return ScaleJoystickTheta_RADPS(rawAxis, maxAngularSpeed.getRadians());
  }

  public static double ScaleJoystickTheta_RADPS(double rawAxis, double maxAngularSpeed_radps) {
    // double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND ? 0 : rawAxis;
    double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND
        ? 0
        : Math.signum(rawAxis) * Math.pow(rawAxis, 2);
    // double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND ? 0 : Math.pow(rawAxis, 3);
    return angular * maxAngularSpeed_radps;
  }
}
