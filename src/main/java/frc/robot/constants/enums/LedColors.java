package frc.robot.constants.enums;

public enum LedColors {
  SPACE_COOKIE(130, 197, 228),
  BLUE(0, 0, 255),
  OFF(0, 0, 0),
  BLACK(0, 0, 0),
  DARK_GREEN(0, 100, 0),
  AZURE(0, 145, 255),
  LESS_LIME(0, 200, 0),
  GREEN(0, 255, 0),
  LIME(0, 255, 0),
  MEDIUM_SPRING_GREEN(0, 255, 145),
  AQUA(0, 255, 255),
  CYAN(0, 255, 255),
  SEA_GREEN(3, 252, 219),
  BLOOD_RED(100, 0, 0),
  CUBE(133, 23, 192),
  PURPLE(133, 23, 192),
  ELECTRIC_VIOLET(145, 0, 255),
  CHARTREUSE(145, 255, 0),
  BLUE_VIOLET(146, 52, 235),
  CADET_GREY(156, 164, 181),
  INCHWORM(188, 237, 81),
  PINK_LAVENDAR(224, 182, 212),
  RED_ORANGE(230, 1, 1),
  RAZZLE_DAZZLE_ROSE(235, 52, 216),
  SAFFRON(235, 180, 52),
  SANDY_BROWN(237, 156, 81),
  PINK(255, 0, 159),
  MAGENTA(255, 0, 145),
  FUSCHIA(255, 0, 255),
  DARK_ORANGE(255, 145, 0),
  CONE(255, 196, 0),
  YELLOW(255, 196, 0),
  WHITE(255, 255, 255);

  public final int r;
  public final int g;
  public final int b;

  LedColors(int r, int g, int b) {
    this.r = r;
    this.g = g;
    this.b = b;
  }
}
