package frc.robot.constants.enums;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import frc.robot.constants.RobotAltModes;
import frc.robot.utils.PIDFConstants;

public enum ModuleControl {
  FALCON_MK4_FALCON_L2(
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(0.6, 0.0, 12.0, 0.0),
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(0.2, 0.0, 0.0, 0.0)
  ),
  FALCON_MK4I_FALCON_L2(
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(1.2, 0.0, 24.0, 0.0),
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(0.4, 0.0, 0.0, 0.0)
  ),
  COLSON_FALCON_MK4_FALCON_L2(
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(0.6, 0.0, 12.0, 0.0),
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(0.2, 0.0, 0.0, 0.0)
  ),
  COLSON_FALCON_MK4I_FALCON_L2(
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(1.2, 0.0, 24.0, 0.0),
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(0.4, 0.0, 0.0, 0.0)
  );
  // https://github.com/FRCTeam2910/2021CompetitionRobot/blob/5fabbff6814a8fa71ef614f691342847ad885bf5/src/main/java/org/frcteam2910/c2020/subsystems/DrivetrainSubsystem.java

  // TODO: Move and factor into an enum or class
  /* Drive Motor Closed Loop PID Values */
  public final PIDFConstants drive;
  /* Steer Motor Closed Loop PID Values */
  public final PIDFConstants steer;
  public final PIDFConstants simDrive;
  public final PIDFConstants simSteer;
  public final TalonFXConfiguration driveConf;
  public final TalonFXConfiguration steerConf;
  public final boolean invertEncoder = false;

  ModuleControl(PIDFConstants drive, PIDFConstants steer) {
    this(drive, steer, drive, steer);
  }

  ModuleControl(
      PIDFConstants drive, PIDFConstants steer, PIDFConstants simDrive, PIDFConstants simSteer
  ) {
    this.drive = drive;
    this.steer = steer;
    this.simDrive = simDrive;
    this.simSteer = simSteer;
    driveConf = initDriveFalcon();
    steerConf = initSteerFalcon();
  }

  private TalonFXConfiguration initDriveFalcon() {
    TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

    // TODO: Move and factor into an enum or class
    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        ElectricalConf.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT,
        ElectricalConf.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT,
        ElectricalConf.DRIVE_PEAK_SUPPLY_CURRENT_LIMIT,
        ElectricalConf.DRIVE_PEAK_CURRENT_DURATION
    );

    driveConfiguration.slot0 =
        RobotAltModes.isSim ? simDrive.toCTRESlotConfiguration() : drive.toCTRESlotConfiguration();
    driveConfiguration.supplyCurrLimit = driveSupplyLimit;
    driveConfiguration.statorCurrLimit = ElectricalConf.DRIVE_STATOR_LIMIT;
    driveConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
    driveConfiguration.openloopRamp = ElectricalConf.DRIVE_OPEN_LOOP_RAMP;
    driveConfiguration.closedloopRamp = ElectricalConf.DRIVE_CLOSED_LOOP_RAMP;

    // TODO: Should this always be on?
    driveConfiguration.voltageCompSaturation =
        ElectricalConf.AUTONOMOUS_MOTOR_VOLTAGE_COMPENSATION_SCALE;
    // TODO: Avoid circular reference
    return driveConfiguration;
  }

  private TalonFXConfiguration initSteerFalcon() {
    TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();

    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        ElectricalConf.ANGLE_ENABLE_CURRENT_LIMIT,
        ElectricalConf.ANGLE_CONTINUOUS_CURRENT_LIMIT,
        ElectricalConf.ANGLE_PEAK_CURRENT_LIMIT,
        ElectricalConf.ANGLE_PEAK_CURRENT_DURATION
    );
    StatorCurrentLimitConfiguration angleStatorLimit = new StatorCurrentLimitConfiguration(
        ElectricalConf.ANGLE_ENABLE_CURRENT_LIMIT,
        ElectricalConf.ANGLE_CONTINUOUS_CURRENT_LIMIT,
        ElectricalConf.ANGLE_PEAK_CURRENT_LIMIT,
        ElectricalConf.ANGLE_PEAK_CURRENT_DURATION
    );
    steerConfiguration.slot0 =
        RobotAltModes.isSim ? simSteer.toCTRESlotConfiguration() : steer.toCTRESlotConfiguration();

    steerConfiguration.supplyCurrLimit = angleSupplyLimit;
    steerConfiguration.statorCurrLimit = angleStatorLimit;
    steerConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;

    steerConfiguration.voltageCompSaturation =
        ElectricalConf.AUTONOMOUS_MOTOR_VOLTAGE_COMPENSATION_SCALE;

    return steerConfiguration;
  }

  /* Neutral Modes */
  public static final NeutralMode STEER_NEUTRAL_MODE = NeutralMode.Coast;
  public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

  // TODO: Move and factor into an enum or class
  public static final class ElectricalConf {
    /* Drive Motor Characterization Values */
    // divide by 12 to convert from volts to percent output for CTRE (not sure if
    // this is needed)
    public static final double DRIVE_KS_VOLT = 0.667 / 12;
    public static final double DRIVE_KV_VOLTPMPS = 2.44 / 12;
    public static final double DRIVE_KA_VOLTPMPS_SQ = 0.27 / 12;
    // /* Drive Motor Characterization Values */
    // public static final SimpleMotorFeedforward driveFeedforward = new
    // SimpleMotorFeedforward(0.2, 2.2201, 0.16343);
    // ???

    /* Swerve Current Limiting */
    public static final double ANGLE_CONTINUOUS_CURRENT_LIMIT = 30;
    public static final double ANGLE_PEAK_CURRENT_LIMIT = 40;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

    public static final double DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = 30;
    public static final double DRIVE_PEAK_SUPPLY_CURRENT_LIMIT = 40;

    public static final double DRIVE_CONTINUOUS_STATOR_CURRENT_LIMIT = 80;
    public static final double DRIVE_PEAK_STATOR_CURRENT_LIMIT = 90;

    public static final double AUTON_DRIVE_CONTINUOUS_STATOR_CURRENT_LIMIT = 60;
    public static final double AUTON_DRIVE_PEAK_STATOR_CURRENT_LIMIT = 80;

    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT = true;
    public static final boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT = true;

    public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
    public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0;

    public static final double AUTONOMOUS_MOTOR_VOLTAGE_COMPENSATION_SCALE = 12.0;

    // TODO: Move and factor into an enum or class
    public static final StatorCurrentLimitConfiguration DRIVE_STATOR_LIMIT =
        new StatorCurrentLimitConfiguration(
            ElectricalConf.DRIVE_ENABLE_STATOR_CURRENT_LIMIT,
            ElectricalConf.DRIVE_CONTINUOUS_STATOR_CURRENT_LIMIT,
            ElectricalConf.DRIVE_PEAK_STATOR_CURRENT_LIMIT,
            ElectricalConf.DRIVE_PEAK_CURRENT_DURATION
        );
    public static final StatorCurrentLimitConfiguration AUTO_DRIVE_STATOR_LIMIT =
        new StatorCurrentLimitConfiguration(
            ElectricalConf.DRIVE_ENABLE_STATOR_CURRENT_LIMIT,
            ElectricalConf.DRIVE_CONTINUOUS_STATOR_CURRENT_LIMIT,
            ElectricalConf.DRIVE_PEAK_STATOR_CURRENT_LIMIT,
            ElectricalConf.DRIVE_PEAK_CURRENT_DURATION
        );
  }
}
