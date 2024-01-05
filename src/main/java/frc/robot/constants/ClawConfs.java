package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Ports.ClawPorts;

public enum ClawConfs {
  NO_CLAW(null, null),
  // CLAW,
  COMP_KLAW(ClawDims.EXTENDED_KLAW, ClawPorts.COMP_PORTS),
  PRACTICE_KLAW(ClawDims.KLAW, ClawPorts.PRACTICE_PORTS),
  // PNEUMATIC_CLAW(),
  MOTOR_KLAW(ClawDims.EXTENDED_KLAW, ClawPorts.WRISTED_KLAW_PORTS);

  public final ClawDims dims;
  public final ClawPorts ports;

  ClawConfs(ClawDims dims, ClawPorts ports) {
    this.dims = dims;
    this.ports = ports;
  }

  public enum ClawDims {
    // CLAW(), // no data
    KLAW(15.0, 15.0),
    EXTENDED_KLAW(24.0, 24.0);

    public final double openWidth_M, halfOpenWidth_M;
    public final double closedWidth_M, halfClosedWidth_M;

    ClawDims(double open_IN, double closed_IN) {
      openWidth_M = Units.inchesToMeters(open_IN);
      halfOpenWidth_M = openWidth_M / 2.0;
      closedWidth_M = Units.inchesToMeters(closed_IN);
      halfClosedWidth_M = closedWidth_M / 2.0;
    }
  }

  public static final DoubleSolenoid.Value CLAW_SOLENOID_CLOSED = DoubleSolenoid.Value.kReverse;
  public static final DoubleSolenoid.Value CLAW_SOLENOID_OPEN = DoubleSolenoid.Value.kForward;
  public static final DoubleSolenoid.Value CLAW_SOLENOID_OFF = DoubleSolenoid.Value.kOff;

  public static final double CLAW_ROLLER_SPINUP_TIME_S = Units.millisecondsToSeconds(80);
  public static final double CLAW_CUBE_SETTLE_TIME_S = Units.millisecondsToSeconds(20);
  public static final double CLAW_CONE_SETTLE_TIME_S = Units.millisecondsToSeconds(60);
  public static final double CLAW_CUBE_STALL_POWER = 0.15;
  public static final double CLAW_CONE_STALL_POWER = 0.29;
  public static final double CLAW_CUBE_AUTO_STALL_POWER = 0.3;
  public static final double CLAW_CUBE_INTAKE_RESETTLE = 0.2;
  public static final double CLAW_CONE_INTAKE_RESETTLE = 0.6;
  public static final int CLAW_SHOT_DELAY_TICKS = 10;
  public static final boolean PRIMARY_ROLLERS_INVERTED = false;
  public static final boolean LEFT_ROLLERS_INVERTED = false;
  public static final NeutralMode ROLLER_NEUTRAL_MODE = NeutralMode.Brake;

  public static final double CLAW_RELEASE_DELAY_S = 0.05;
  public static final double CLAW_DEPLOY_IN_TIME_MS = 500;
  public static final double CLAW_DEPLOY_IN_TIME_S = 0.5;
  public static final double CLAW_DEPLOY_OUT_TIME_MS = 500;
  public static final double CLAW_DEPLOY_OUT_TIME_S = 0.5;

  public static final double CLAW_SETTLING_WAIT_MS = 150;
  public static final double CLAW_PLATFORM_CLEARANCE_WAIT_S = 0.8;
  public static final double CLAW_OPEN_PLATFORM_PREP_DELAY_S = 0.5;

  /* Claw PID Values */
  public static final double CLAW_PFAC = 0.0;
  public static final double CLAW_IFAC = 0.0;
  public static final double CLAW_DFAC = 0.0;
  public static final double CLAW_FFAC = 0.0;

  /* Claw Supply Current Limiting */
  public static final int CLAW_SUPPLY_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final int CLAW_SUPPLY_PEAK_CURRENT_LIMIT = 40;
  public static final double CLAW_SUPPLY_PEAK_CURRENT_DURATION_S = 0.1;
  public static final int CLAW_SUPPLY_PEAK_CURRENT_DURATION_MS =
      (int) (1000.0 * CLAW_SUPPLY_PEAK_CURRENT_DURATION_S);
  public static final boolean CLAW_SUPPLY_ENABLE_CURRENT_LIMIT = true;

  /* Claw Stator Current Limiting */
  public static final int CLAW_STATOR_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final int CLAW_STATOR_PEAK_CURRENT_LIMIT = 40;
  public static final double CLAW_STATOR_PEAK_CURRENT_DURATION_S = 0.1;
  public static final int CLAW_STATOR_PEAK_CURRENT_DURATION_MS =
      (int) (1000.0 * CLAW_STATOR_PEAK_CURRENT_DURATION_S);

  public static final boolean CLAW_STATOR_ENABLE_CURRENT_LIMIT = true;

  // /* Claw Rollers PID Values */
  // public static final double CLAW_ROLLERS_PFAC = 0.0;
  // public static final double CLAW_ROLLERS_IFAC = 0.0;
  // public static final double CLAW_ROLLERS_DFAC = 0.0;
  // public static final double CLAW_ROLLERS_FFAC = 0.0;

  /* Claw Rollers Supply Current Limiting */
  public static final int CLAW_ROLLERS_SUPPLY_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final int CLAW_ROLLERS_SUPPLY_PEAK_CURRENT_LIMIT = 40;
  public static final double CLAW_ROLLERS_SUPPLY_PEAK_CURRENT_DURATION_S = 0.1;
  public static final int CLAW_ROLLERS_SUPPLY_PEAK_CURRENT_DURATION_MS =
      (int) (1000.0 * CLAW_ROLLERS_SUPPLY_PEAK_CURRENT_DURATION_S);
  public static final boolean CLAW_ROLLERS_SUPPLY_ENABLE_CURRENT_LIMIT = true;

  /* Claw Rollers Stator Current Limiting */
  public static final int CLAW_ROLLERS_STATOR_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final int CLAW_ROLLERS_STATOR_PEAK_CURRENT_LIMIT = 40;
  public static final double CLAW_ROLLERS_STATOR_PEAK_CURRENT_DURATION_S = 0.1;
  public static final int CLAW_ROLLERS_STATOR_PEAK_CURRENT_DURATION_MS =
      (int) (1000.0 * CLAW_ROLLERS_STATOR_PEAK_CURRENT_DURATION_S);
  public static final boolean CLAW_ROLLERS_STATOR_ENABLE_CURRENT_LIMIT = true;

  public static final SupplyCurrentLimitConfiguration clawWristSupplyLimit =
      new SupplyCurrentLimitConfiguration(
          CLAW_SUPPLY_ENABLE_CURRENT_LIMIT,
          CLAW_SUPPLY_CONTINUOUS_CURRENT_LIMIT,
          CLAW_SUPPLY_PEAK_CURRENT_LIMIT,
          CLAW_SUPPLY_PEAK_CURRENT_DURATION_S
      );
  public static final StatorCurrentLimitConfiguration clawWristStatorLimit =
      new StatorCurrentLimitConfiguration(
          CLAW_STATOR_ENABLE_CURRENT_LIMIT,
          CLAW_STATOR_CONTINUOUS_CURRENT_LIMIT,
          CLAW_STATOR_PEAK_CURRENT_LIMIT,
          CLAW_STATOR_PEAK_CURRENT_DURATION_S
      );

  public static final SupplyCurrentLimitConfiguration clawRollerSupplyLimit =
      new SupplyCurrentLimitConfiguration(
          CLAW_ROLLERS_SUPPLY_ENABLE_CURRENT_LIMIT,
          CLAW_ROLLERS_SUPPLY_CONTINUOUS_CURRENT_LIMIT,
          CLAW_ROLLERS_SUPPLY_PEAK_CURRENT_LIMIT,
          CLAW_ROLLERS_SUPPLY_PEAK_CURRENT_DURATION_S
      );

  public static final StatorCurrentLimitConfiguration clawRollerStatorLimit =
      new StatorCurrentLimitConfiguration(
          CLAW_ROLLERS_STATOR_ENABLE_CURRENT_LIMIT,
          CLAW_ROLLERS_STATOR_CONTINUOUS_CURRENT_LIMIT,
          CLAW_ROLLERS_STATOR_PEAK_CURRENT_LIMIT,
          CLAW_ROLLERS_STATOR_PEAK_CURRENT_DURATION_S
      );

  public static final double CLAW_OPEN_LOOP_RAMP = 0.0;
  public static final double CLAW_CLOSED_LOOP_RAMP = 0.0;

  public static final class IntakeSpeed {
    public static double CUBE_POWER;
    public static double CONE_POWER;
    public static double CONE_DETECTION_CURRENT_THRESHOLD_A;

    static {
      if (RobotAltModes.isTestMode) {
        CONE_DETECTION_CURRENT_THRESHOLD_A = 23.0;
        CUBE_POWER = 0.3 * RobotAltModes.TEST_MODE_COEFFICIENT;
        CONE_POWER = 0.6 * RobotAltModes.TEST_MODE_COEFFICIENT;
      } else {
        CONE_DETECTION_CURRENT_THRESHOLD_A = 23.0;
        CONE_POWER = 0.84;
        CUBE_POWER = 0.575;
      }
    }
  }
}
