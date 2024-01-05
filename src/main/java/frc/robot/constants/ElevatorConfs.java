package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports.ElevatorPorts;

public enum ElevatorConfs {
  COMP_BOT_CONF(ElevatorPorts.COMP_PORTS, ElevatorDims.COMP_ELEVATOR, ElevatorControl.COMP_BOT),
  PRACTICE_BOT_CONF(
      ElevatorPorts.PRACTICE_PORTS, ElevatorDims.PRACTICE_ELEVATOR, ElevatorControl.PRACTICE_BOT
  );

  public final ElevatorPorts ports;
  public final ElevatorDims dims;
  public final ElevatorControl control;

  ElevatorConfs(ElevatorPorts ports, ElevatorDims dims, ElevatorControl control) {
    this.ports = ports;
    this.dims = dims;
    this.control = control;
  }

  public enum ElevatorDims {
    COMP_ELEVATOR(),
    PRACTICE_ELEVATOR();

    public static final int ELEVATOR_INCLINE_ANGLE_DEG = 50;
    // 1.114" pitch diameter
    public static final double ELEVATOR_DRUM_RADIUS_M = Units.inchesToMeters(0.6365);
    public static final double ELEVATOR_CIRCUMFERENCE_M = ELEVATOR_DRUM_RADIUS_M * 2 * Math.PI;
    // TODO ALL
    // 12:60 gear ratio on main stage
    public static final double ELEVATOR_GEAR_RATIO = 5.0;
    // 7 lb claw mechanism, 15-20 lbs
    public static final double ELEVATOR_WEIGHT_LB = 7.2;
    public static final double ELEVATOR_MIN_HEIGHT_M = 0;
    public static final double ELEVATOR_MAX_HEIGHT_M = Units.inchesToMeters(35.0);
  }

  public enum ElevatorControl { // values after elevator tuning
    COMP_BOT(new double[] {0.355, 6.67, 0.05, 0.625}, new double[] {1.0, 2.0}),
    PRACTICE_BOT(new double[] {0.0, 6.5, 0.0, 0.4}, new double[] {2.0, 2.0});

    /* Elevator Characterization Values */
    public final double ELEVATOR_KS_V;
    public final double ELEVATOR_KV_V_PER_MPS;
    public final double ELEVATOR_KA_V_PER_MPSQ;
    public final double ELEVATOR_KG_V;

    /* Elevator Trapezoidal Profile Limits */
    public final double ELEVATOR_MAX_VELO_MPS;
    public final double ELEVATOR_MAX_ACC_MPSQ;

    /* Elevator PID Configuration */
    public static final double ELEVATOR_PFAC = 0.15;
    public static final double ELEVATOR_IFAC = 0.0;
    public static final double ELEVATOR_DFAC = 0.4;
    public static final double ELEVATOR_FFAC = 0.0;

    public static final double ELEVATOR_OPEN_LOOP_RAMP = 0.0;
    public static final double ELEVATOR_CLOSED_LOOP_RAMP = 0.0;
    public static final double ELEVATOR_VOLTAGE_COMP_SATURATION = 12.0;

    ElevatorControl(double[] characterization, double[] trapezoidalLimits) {
      ELEVATOR_KS_V = characterization[0];
      ELEVATOR_KV_V_PER_MPS = characterization[1];
      ELEVATOR_KA_V_PER_MPSQ = characterization[2];
      ELEVATOR_KG_V = characterization[3];

      ELEVATOR_MAX_VELO_MPS = trapezoidalLimits[0];
      ELEVATOR_MAX_ACC_MPSQ = trapezoidalLimits[1];
    }
  }

  public static final int ELEVATOR_CLEAR_OF_INTAKE_TICKS = 4;
  // TODO Tune....
  public static final int ELEVATOR_TOLERANCE_TICKS = 300;

  public static final boolean ELEVATOR_MOTOR_INVERTED = true;
  public static final boolean ELEVATOR_FOLLOWER_MOTOR_INVERTED = !ELEVATOR_MOTOR_INVERTED;
  public static final NeutralMode ELEVATOR_NEUTRAL_MODE = NeutralMode.Coast;
  public static final NeutralMode ELEVATOR_DISABLED_MODE = NeutralMode.Brake;

  public static final boolean ELEVATOR_SUPPLY_ENABLE_CURRENT_LIMIT = true;
  public static final int ELEVATOR_SUPPLY_CONTINUOUS_CURRENT_LIMIT = 60;
  public static final int ELEVATOR_SUPPLY_PEAK_CURRENT_LIMIT = 60;
  public static final double ELEVATOR_SUPPLY_PEAK_CURRENT_DURATION = 0.1;

  public static final boolean ELEVATOR_STATOR_ENABLE_CURRENT_LIMIT = true;
  public static final int ELEVATOR_STATOR_CONTINUOUS_CURRENT_LIMIT = 60;
  public static final int ELEVATOR_STATOR_PEAK_CURRENT_LIMIT = 60;
  public static final double ELEVATOR_STATOR_PEAK_CURRENT_DURATION = 0.1;
  public static final double ELEVATOR_MOTOR_DEADBAND = 0.001;

  public static final int ELEVATOR_ZEROING_STALL_TOLERANCE = 50;
  public static final int ELEVATOR_ZEROING_STALL_LOOPS = 6;
  public static final double ELEVATOR_ZEROING_SPEED_MPS = -0.06;
}
