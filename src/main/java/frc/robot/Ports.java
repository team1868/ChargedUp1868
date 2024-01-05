package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Ports {
  public static final int DRIVER_XBOX_USB_PORT = 0;
  public static final int OPERATOR_PS4_USB_PORT = 1;
  public static final int PIT_XBOX_USB_PORT = 2;

  public enum ClawPorts {
    COMP_PORTS(24, 25, 9),
    PRACTICE_PORTS(24, 25, 9),
    WRISTED_KLAW_PORTS(24, 25, 9);

    public final int leftRoller;
    public final int rightRoller;
    public final int breakBeam;

    // Unused until wrist motorized
    // public final int wrist;
    // public final int wristEncoder;

    ClawPorts(int leftRoller, int rightRoller, int breakBeam) {
      this.leftRoller = leftRoller;
      this.rightRoller = rightRoller;
      this.breakBeam = breakBeam;
    }
  }

  public enum DrivetrainPorts {
    COMP_PORTS(
        "Default Name",
        new int[] {10, 16, 12, 14},
        new int[] {11, 17, 13, 15},
        new int[] {1, 3, 0, 2},
        "Default Name",
        35
    ),
    PRACTICE_PORTS(
        "Default Name",
        new int[] {10, 16, 12, 14},
        new int[] {11, 17, 13, 15},
        new int[] {1, 2, 0, 3},
        "Default Name",
        35
    ),
    SWERVE_BASE_PORTS(
        "rio",
        new int[] {11, 16, 12, 14},
        new int[] {10, 17, 13, 15},
        new int[] {30, 33, 31, 32},
        "rio",
        35
    );

    public final String moduleCanBus;
    public final int[] drive;
    public final int[] steer;
    public final int[] encoder;
    public final String pigeonCanBus;
    public final int pigeonID;

    DrivetrainPorts(
        String moduleCanBus,
        int[] drive,
        int[] steer,
        int[] encoder,
        String pigeonCanBus,
        int pigeonID
    ) {
      this.moduleCanBus = moduleCanBus;
      this.drive = drive;
      this.steer = steer;
      this.encoder = encoder;
      this.pigeonCanBus = pigeonCanBus;
      this.pigeonID = pigeonID;
    }
  }

  public enum ElevatorPorts {
    COMP_PORTS(20, 21, -1),
    PRACTICE_PORTS(20, 21, -1);

    public final int primaryMotor;
    public final int followerMotor;
    public final int limitSwitch;

    ElevatorPorts(int primary, int follower, int limitSwitch) {
      primaryMotor = primary;
      followerMotor = follower;
      this.limitSwitch = limitSwitch;
    }
  }

  public enum LedPorts {
    COMP_PORTS("", 50),
    PRACTICE_PORTS("", 50),
    SWERVE_BASE_PORTS("", 50);

    public final int candle;
    public final String candleCanBus;

    LedPorts(String candleCanBus, int candle) {
      this.candle = candle;
      this.candleCanBus = candleCanBus;
    }
  }

  public enum PneumaticPorts {
    COMP_PORTS(PneumaticsModuleType.CTREPCM, 42, 1, 6, 0, 5),
    PRACTICE_PORTS(PneumaticsModuleType.CTREPCM, 42, 4, 2, 1, 3);

    public final PneumaticsModuleType moduleType;
    public final int moduleID;
    public final int clawForward, clawReverse;
    public final int intakeForward, intakeReverse;

    PneumaticPorts(
        PneumaticsModuleType type,
        int moduleID,
        int clawForward,
        int clawReverse,
        int intakeForward,
        int intakeReverse
    ) {
      this.moduleType = type;
      this.moduleID = moduleID;
      this.clawForward = clawForward;
      this.clawReverse = clawReverse;
      this.intakeForward = intakeForward;
      this.intakeReverse = intakeReverse;
    }
  }
}
