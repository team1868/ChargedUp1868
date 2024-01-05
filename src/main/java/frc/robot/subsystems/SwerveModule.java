package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConfs;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.ModuleControl;
import frc.robot.utils.CTREConversion;
import frc.robot.utils.CTREModuleState;

public class SwerveModule {
  public static final DrivetrainConfs DRIVE_CONSTS = Constants.CRobot.drive;

  /* --- Module Identifiers --- */
  public final int _moduleNumber;
  public final String _moduleNumberStr;

  /* --- Sensors, motors, and hardware --- */
  private final WPI_TalonFX _angleMotor;
  private final WPI_TalonFX _driveMotor;

  private AnalogInput _analogInput;
  private AnalogPotentiometer _angleEncoder;
  private WPI_CANCoder _angleEncoderCAN;

  /* --- State and Physical Property variables --- */
  private SwerveModuleState _desiredState;
  private SwerveModuleState _curState = new SwerveModuleState();
  private SwerveModulePosition _curPosition = new SwerveModulePosition();
  private final double _angleOffsetDeg;
  private double _lastAngleDeg;
  private double _percentOutput = 0.0;
  private double _velocity = 0.0;
  private double _angleDeg = 0.0;
  private double _absolutePosition;
  private final String _canBusName;

  /* --- Control Utils --- */
  private static CANCoderConfiguration _swerveCanCoderConfig;
  private static StatorCurrentLimitConfiguration _driveStatorLimit;
  private static StatorCurrentLimitConfiguration _autonomousDriveStatorLimit;

  // TODO: Move constants but not class into separate enum or configuration
  private final SimpleMotorFeedforward _feedforward = new SimpleMotorFeedforward(
      ModuleControl.ElectricalConf.DRIVE_KS_VOLT,
      ModuleControl.ElectricalConf.DRIVE_KV_VOLTPMPS,
      ModuleControl.ElectricalConf.DRIVE_KA_VOLTPMPS_SQ
  );

  /* --- Simulation resources and variables --- */
  private TalonFXSimCollection _driveMotorSim;
  private TalonFXSimCollection _angleMotorSim;

  private FlywheelSim _driveWheelSim;

  private SingleJointedArmSim _moduleAngleSim;

  /* --- Shuffleboard entries --- */
  public GenericEntry _steerAnglePFac, _steerAngleIFac, _steerAngleDFac;
  public GenericEntry _steerPFac, _steerIFac, _steerDFac;

  // telemetry widgets
  public GenericEntry _shuffleboardModuleCANcoder;
  public GenericEntry _shuffleboardModuleAngle;
  public GenericEntry _shuffleboardModuleSpeed;

  public SwerveModule(int moduleNumber, String canBusName) {
    _moduleNumber = moduleNumber;
    _canBusName = canBusName;
    _moduleNumberStr = "M" + Integer.toString(_moduleNumber);
    _angleOffsetDeg = DRIVE_CONSTS.moduleOffsetsDeg[_moduleNumber];
    initCanCoderConfigs();

    _angleMotor = new WPI_TalonFX(DRIVE_CONSTS.ports.steer[_moduleNumber], canBusName);
    _driveMotor = new WPI_TalonFX(DRIVE_CONSTS.ports.drive[_moduleNumber], canBusName);

    waitForCAN();

    // Must be done before the angle motor is configured
    configAngleEncoder();

    configAngleMotor();

    configDriveMotor();

    configShuffleboard();

    // probably redundant
    resetToAbsolute();

    simulationInit();
  }

  public SwerveModule(int moduleNumber) {
    this(moduleNumber, "");
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    setDesiredState(desiredState, isOpenLoop, false);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean is254) {
    if (_angleMotor.hasResetOccurred()) {
      resetToAbsolute();
    }

    // TODO cleanup
    _desiredState = is254 ? CTREModuleState.optimize254(desiredState, getState().angle)
                          : CTREModuleState.optimize(desiredState, getState().angle);
    // _desiredState = desiredState;

    if (isOpenLoop) {
      _percentOutput = _desiredState.speedMetersPerSecond
          / Constants.CRobot.drive.model.theoreticalMaxWheelSpeed;
      _driveMotor.set(ControlMode.PercentOutput, _percentOutput);
    } else {
      _velocity = CTREConversion.MPSToFalcon(
          _desiredState.speedMetersPerSecond,
          DRIVE_CONSTS.type.wheelCircumferenceM,
          DRIVE_CONSTS.model.driveRatio
      );
      _driveMotor.set(
          ControlMode.Velocity,
          _velocity,
          DemandType.ArbitraryFeedForward,
          _feedforward.calculate(_desiredState.speedMetersPerSecond)
      );
    }

    _angleDeg = Math.abs(_desiredState.speedMetersPerSecond)
            <= (Constants.CRobot.drive.model.theoreticalMaxWheelSpeed * 0.01)
        ? _lastAngleDeg
        : _desiredState.angle.getDegrees();
    _angleMotor.set(
        ControlMode.Position,
        CTREConversion.degreesToFalcon(_angleDeg, DRIVE_CONSTS.model.steerRatio)
    );
    _lastAngleDeg = _angleDeg;
  }

  public double getDegreeAngleEncoder() {
    if (Constants.isCANEncoder) {
      return _angleEncoderCAN.getAbsolutePosition();
    }
    return _angleEncoder.get() + _angleOffsetDeg;
  }

  public SwerveModuleState getState() {
    _curState.speedMetersPerSecond = CTREConversion.falconToMPS(
        _driveMotor.getSelectedSensorVelocity(),
        DRIVE_CONSTS.type.wheelCircumferenceM,
        DRIVE_CONSTS.model.driveRatio
    );
    _curState.angle = CTREConversion.falconToRotation2d(
        _angleMotor.getSelectedSensorPosition(), DRIVE_CONSTS.model.steerRatio
    );

    return _curState;
  }

  public SwerveModulePosition getPosition() {
    _curPosition.distanceMeters = CTREConversion.falconToMeters(
        _driveMotor.getSelectedSensorPosition(),
        DRIVE_CONSTS.type.wheelCircumferenceM,
        DRIVE_CONSTS.model.driveRatio
    );
    _curPosition.angle = CTREConversion.falconToRotation2d(
        _angleMotor.getSelectedSensorPosition(), DRIVE_CONSTS.model.steerRatio
    );
    // might be necessary
    // Rotation2d.fromDegrees(CTREConversion.falconToDegrees(
    //     _angleMotor.getSelectedSensorPosition(), DRIVE_CONSTS.model.steerRatio
    // ));

    return _curPosition;
  }

  private void waitForCAN() {
    if (Robot.isReal()) {
      int counter = 0;
      String identifier = "SWERVE " + _moduleNumberStr + " Check Init Status : ";
      while (!checkInitStatus()) {
        System.out.println(identifier + counter++);
      }
    } else {
    }
  }

  public void resetToAbsolute() {
    _lastAngleDeg = getDegreeAngleEncoder();
    _absolutePosition =
        CTREConversion.degreesToFalcon(_lastAngleDeg, DRIVE_CONSTS.model.steerRatio);
    _angleMotor.setSelectedSensorPosition(_absolutePosition);
  }

  public void configShuffleboard() {
    if (RobotAltModes.isVerboseMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("SwerveModules");
      ShuffleboardLayout layout = tab.getLayout(_moduleNumberStr, BuiltInLayouts.kGrid)
                                      .withSize(1, 3)
                                      .withPosition(_moduleNumber, 0);

      _shuffleboardModuleAngle = layout.add("angleState", 0.0).withPosition(0, 0).getEntry();
      _shuffleboardModuleCANcoder = layout.add("angleCANcoder", 0.0).withPosition(0, 1).getEntry();
      _shuffleboardModuleSpeed = layout.add("speed", 0.0).withPosition(0, 2).getEntry();
    }
  }

  public void updateShuffleboard() {
    if (RobotAltModes.isVerboseMode) {
      _shuffleboardModuleAngle.setDouble(_curState.angle.getDegrees());
      _shuffleboardModuleCANcoder.setDouble(getDegreeAngleEncoder());
      _shuffleboardModuleSpeed.setDouble(_curState.speedMetersPerSecond);
    }
  }

  public void alignToZero() {
    resetToAbsolute();
    _lastAngleDeg = 0;
  }

  public boolean motorResetConfig() {
    boolean result = false;
    if (Constants.isCANEncoder && _angleEncoderCAN.hasResetOccurred()) {
      configCanEncoder();
      result = true;
    }
    if (_angleMotor.hasResetOccurred()) {
      configAngleMotor();
      result = true;
    }
    if (_driveMotor.hasResetOccurred()) {
      configDriveMotor();
      result = true;
    }
    return result;
  }

  public void configPID(double ap, double ai, double ad, double p, double i, double d) {
    if (RobotAltModes.isPIDTuningMode) {
      Constants.CRobot.drive.moduleControl.steerConf.slot0.kP = ap;
      Constants.CRobot.drive.moduleControl.steerConf.slot0.kI = ai;
      Constants.CRobot.drive.moduleControl.steerConf.slot0.kD = ad;

      Constants.CRobot.drive.moduleControl.driveConf.slot0.kP = p;
      Constants.CRobot.drive.moduleControl.driveConf.slot0.kI = i;
      Constants.CRobot.drive.moduleControl.driveConf.slot0.kD = d;

      configAngleMotor();
      configDriveMotor();
    }
  }

  public void setLastAngle(SwerveModuleState desiredState) {
    SwerveModuleState goalModuleState = CTREModuleState.optimize(desiredState, getState().angle);
    _lastAngleDeg = goalModuleState.angle.getDegrees();
  }

  public void setLastAngle(Rotation2d angle) {
    setLastAngleDeg(angle.getDegrees());
  }

  public void setLastAngleDeg(double angle_deg) {
    _lastAngleDeg = angle_deg;
  }

  public boolean isAlignedTo(SwerveModuleState goalState, double tolerance_deg) {
    return Math.abs(_angleDeg - goalState.angle.getDegrees()) < tolerance_deg;
  }

  public boolean isAlignedTo(SwerveModuleState goalState) {
    return isAlignedTo(goalState, DRIVE_CONSTS.alignmentToleranceDeg);
  }

  public void autonomousDriveMode(boolean enable) {
    _driveMotor.enableVoltageCompensation(enable);
    _driveMotor.configStatorCurrentLimit(
        enable ? ModuleControl.ElectricalConf.AUTO_DRIVE_STATOR_LIMIT
               : ModuleControl.ElectricalConf.DRIVE_STATOR_LIMIT
    );
    _angleMotor.enableVoltageCompensation(enable);
  }

  private void configAngleEncoder() {
    if (Constants.isCANEncoder) {
      initCanEncoder();
      configCanEncoder();
    } else {
      configAnalogEncoder();
    }
  }

  private void initCanEncoder() {
    _angleEncoderCAN = new WPI_CANCoder(DRIVE_CONSTS.ports.encoder[_moduleNumber], _canBusName);
  }

  private void configCanEncoder() {
    _angleEncoderCAN.configFactoryDefault(100);
    _angleEncoderCAN.configAllSettings(_swerveCanCoderConfig);
    _angleEncoderCAN.configMagnetOffset(_angleOffsetDeg);
  }

  private void configAnalogEncoder() {
    _analogInput = new AnalogInput(DRIVE_CONSTS.ports.encoder[_moduleNumber]);
    _angleEncoder = new AnalogPotentiometer(_analogInput, 360.0);
    _analogInput.setAverageBits(2);
  }

  private void configAngleMotor() {
    _angleMotor.configFactoryDefault();
    _angleMotor.configAllSettings(Constants.CRobot.drive.moduleControl.steerConf);
    _angleMotor.setInverted(Constants.CRobot.drive.model.type.invertSteer);
    _angleMotor.setNeutralMode(ModuleControl.STEER_NEUTRAL_MODE);

    resetToAbsolute();
  }

  private void configDriveMotor() {
    _driveMotor.configFactoryDefault();
    _driveMotor.configAllSettings(Constants.CRobot.drive.moduleControl.driveConf);
    _driveMotor.setInverted(Constants.CRobot.drive.model.type.invertDrive);
    _driveMotor.setNeutralMode(ModuleControl.DRIVE_NEUTRAL_MODE);
    _driveMotor.setSelectedSensorPosition(0);
  }

  private boolean checkInitStatus() {
    return _angleMotor.configFactoryDefault() == ErrorCode.OK;
  }

  public void simulationInit() {
    if (!Robot.isReal())
      _driveMotorSim = _driveMotor.getSimCollection();
    _angleMotorSim = _angleMotor.getSimCollection();

    _driveWheelSim = new FlywheelSim(
        DCMotor.getFalcon500(1),
        DRIVE_CONSTS.model.driveRatio,
        0.01,
        VecBuilder.fill(2.0 * Math.PI / 2048)
    );

    _moduleAngleSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        DRIVE_CONSTS.model.steerRatio,
        0.001,
        0.0,
        -Double.MAX_VALUE,
        Double.MAX_VALUE,
        false,
        VecBuilder.fill(2.0 * Math.PI / 2048)
    );
  }

  public void simulationPeriodic() {
    if (RobotAltModes.isSim) {
      // Update the model motor sim _driveWheelSim, and read its angular velocity
      _driveWheelSim.setInputVoltage(_driveMotor.getMotorOutputVoltage());
      _driveWheelSim.update(Constants.LOOP_PERIOD_MS);

      double driveSimOmega = _driveWheelSim.getAngularVelocityRPM();
      int driveTicksPer100ms = (_driveMotor.getInverted() ? -1 : 1)
          * CTREConversion.RPMToFalcon(driveSimOmega, DRIVE_CONSTS.model.driveRatio);

      // Update integrated sensor sim in _driveMotor
      _driveMotorSim.setIntegratedSensorVelocity(driveTicksPer100ms);
      _driveMotorSim.addIntegratedSensorPosition((int
      ) (driveTicksPer100ms * Constants.LOOP_PERIOD_MS / 100.0));

      // Update _steeringSim single-arm simulation
      _moduleAngleSim.setInputVoltage(_angleMotor.getMotorOutputVoltage());
      _moduleAngleSim.update(Constants.LOOP_PERIOD_MS);

      // Update integratedSensor sim in _angleMotor
      int angleSign = _angleMotor.getInverted() ? -1 : 1;
      _angleMotorSim.setIntegratedSensorVelocity(
          angleSign
          * CTREConversion.RPMToFalcon(
              Units.radiansPerSecondToRotationsPerMinute(_moduleAngleSim.getVelocityRadPerSec()),
              DRIVE_CONSTS.model.steerRatio
          )
      );
      _angleMotorSim.setIntegratedSensorRawPosition(
          angleSign
          * CTREConversion.radiansToFalcon(
              _moduleAngleSim.getAngleRads(), DRIVE_CONSTS.model.steerRatio
          )
      );
    }
  }

  private static void initCanCoderConfigs() {
    if (Constants.isCANEncoder && _swerveCanCoderConfig == null) {
      /* Swerve CANCoder Configuration */
      _swerveCanCoderConfig = new CANCoderConfiguration();

      _swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
      _swerveCanCoderConfig.sensorDirection = Constants.CRobot.drive.moduleControl.invertEncoder;
      _swerveCanCoderConfig.initializationStrategy =
          SensorInitializationStrategy.BootToAbsolutePosition;
      _swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
  }

  // Ryan's 254 work
  public double getUnclampedSteerAngleRadians() {
    return (_angleMotor.getSelectedSensorPosition() * kSteerPositionCoefficient)
        - Units.degreesToRadians(_angleOffsetDeg);
  }

  private void setSteerAngleUnclamped(double steerAngleRadians) {
    _angleMotor.set(
        TalonFXControlMode.Position,
        (steerAngleRadians + Units.degreesToRadians(_angleOffsetDeg)) / kSteerPositionCoefficient
    );
  }

  public void setWithVoltageShortestPath(double drivePercentage, Rotation2d steerAngle) {
    final boolean flip = setSteerAngleShortestPath(steerAngle);
    _driveMotor.set(TalonFXControlMode.PercentOutput, flip ? -drivePercentage : drivePercentage);
  }

  public void setWithVelocityShortestPath(double driveVelocity, Rotation2d steerAngle) {
    final boolean flip = setSteerAngleShortestPath(steerAngle);

    _velocity = CTREConversion.MPSToFalcon(
        driveVelocity, DRIVE_CONSTS.type.wheelCircumferenceM, DRIVE_CONSTS.model.driveRatio
    );
    _driveMotor.set(
        TalonFXControlMode.Velocity, (flip ? -_velocity : _velocity) / (kDriveVelocityCoefficient)
    );
  }

  private final double kDrivePositionCoefficient = Math.PI
      * Constants.CRobot.drive.model.type.wheelDiameterM * Constants.CRobot.drive.model.driveRatio
      / 2048.0;
  private final double kDriveVelocityCoefficient =
      kDrivePositionCoefficient * Constants.LOOP_PERIOD_MS;
  private final double kSteerPositionCoefficient =
      2.0 * Math.PI / 2048.0 * Constants.CRobot.drive.model.steerRatio;

  public void setWithVoltageUnclamped(double drivePercentage, double steerAngleRadians) {
    setSteerAngleUnclamped(steerAngleRadians);
    _driveMotor.set(TalonFXControlMode.PercentOutput, drivePercentage);
  }

  public void setWithVelocityUnclamped(double driveVelocity, double steerAngleRadians) {
    setSteerAngleUnclamped(steerAngleRadians);
    _driveMotor.set(TalonFXControlMode.Velocity, driveVelocity / kDriveVelocityCoefficient);
  }

  // Returns true if the drive velocity should be inverted.
  private boolean setSteerAngleShortestPath(Rotation2d steerAngle) {
    boolean flip = false;
    final double unclampedPosition = getUnclampedSteerAngleRadians();
    final Rotation2d clampedPosition = Rotation2d.fromRadians(unclampedPosition);
    final Rotation2d relativeRotation = steerAngle.rotateBy(clampedPosition.unaryMinus());
    double relativeRadians = relativeRotation.getRadians();
    final double kPiOver2 = Math.PI / 2.0;
    if (relativeRadians > kPiOver2) {
      // Flipping drive direction would be the shorter path.
      flip = true;
      relativeRadians -= Math.PI;
    } else if (relativeRadians < -kPiOver2) {
      // Flipping drive direction would be the shorter path.
      flip = true;
      relativeRadians += Math.PI;
    }
    setSteerAngleUnclamped(unclampedPosition + relativeRadians);

    return flip;
  }
}
