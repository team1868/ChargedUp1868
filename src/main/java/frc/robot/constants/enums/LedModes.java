package frc.robot.constants.enums;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import frc.robot.constants.LedConfs;

public enum LedModes {
  COLOR_FLOW(new ColorFlowAnimation(255, 0, 0, 150, 1, -1, Direction.Forward, 0)),
  FIRE(new FireAnimation(0.7, 0.1, -1, 1, 1, false, 0)),
  LARSON(new LarsonAnimation(255, 0, 0, 150, 1, -1, BounceMode.Front, 2, 0)),
  RAINBOW(new RainbowAnimation(0.7, 0.6, -1)),
  RGB_FADE(new RgbFadeAnimation()),
  BLINK_CONE(new StrobeAnimation(
      LedColors.CONE.r, LedColors.CONE.g, LedColors.CONE.b, LedConfs.LED_WHITE_LEVEL, 0.6, -1, 0
  )),
  BLINK_CUBE(new StrobeAnimation(
      LedColors.CUBE.r, LedColors.CUBE.g, LedColors.CUBE.b, LedConfs.LED_WHITE_LEVEL, 0.6, -1, 0
  ));

  public final Animation animation;

  LedModes(Animation animation) {
    this.animation = animation;
  }

  public LedModes getDecr() {
    int prev = (this == COLOR_FLOW ? LedModes.values().length : ordinal()) - 1;
    return LedModes.values()[prev];
  }
}
