package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConfs;
import frc.robot.constants.ElevatorConfs.ElevatorControl;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.utils.CTREConversion;
import frc.robot.utils.LoopTimer;

public class Elevator extends SubsystemBase {
  /* --- Constant configuration shortcuts --- */
  public final ElevatorConfs ELEVATOR_CONSTS = Constants.CRobot.elevator;

  /* --- Sensors, motors, and hardware --- */
  private final WPI_TalonFX _elevatorMotor;
  private final WPI_TalonFX _followerMotor;

  /* --- Robot State and Physical Properties variables --- */
  private boolean _followingProfile = false;
  private Timer _timer = new Timer();
  private double _prevVelocity_MPS;
  private ElevatorPositions _goalPosition = ElevatorPositions.ELEVATOR_UNKNOWN_POSITION;
  private int _clampedPosition = ElevatorPositions.ELEVATOR_UNKNOWN_POSITION.ticks;

  /* --- Control Utils --- */
  private TrapezoidProfile _profile;
  private ElevatorFeedforward _elevatorFeedForward;

  private double _prevVelo;

  private TalonFXConfiguration _elevatorMotorFXConfig = new TalonFXConfiguration();

  /* --- Shuffleboard Entries --- */
  private GenericSubscriber _pFactorEntry, _iFactorEntry, _dFactorEntry;
  private GenericSubscriber _kSEntry, _kGEntry, _kVEntry, _kAEntry;
  private GenericPublisher _currentTicksEntry, _targetTicksEntry;
  private GenericPublisher _elevatorCurrentVelocityEntry, _elevatorTargetVelocityEntry;
  private GenericPublisher _elevatorProfilePositionEntry;
  private GenericPublisher _elevatorAccelEntry;
  private GenericPublisher _elevatorPositionErrorEntry, _elevatorVelocityErrorEntry;
  private GenericPublisher _elevatorPercentEntry, _statorCurrentEntry, _supplyCurrentEntry;
  private GenericEntry _elevatorPowerEntry;
  private GenericSubscriber _maxVeloEntry, _maxAccelEntry;

  /* --- Simulation Resources --- */
  private Mechanism2d _mech2d;
  private MechanismRoot2d _elevatorRaise;
  private MechanismLigament2d _elevatorTower;
  private MechanismLigament2d _elevatorStage2;

  static {}

  public Elevator() {
    if (Constants.hasElevator) {
      initMotorConfigs();

      _elevatorMotor = new WPI_TalonFX(ELEVATOR_CONSTS.ports.primaryMotor);
      _followerMotor = new WPI_TalonFX(ELEVATOR_CONSTS.ports.followerMotor);

      _profile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ELEVATOR_CONSTS.control.ELEVATOR_MAX_VELO_MPS,
              ELEVATOR_CONSTS.control.ELEVATOR_MAX_ACC_MPSQ
          ),
          new TrapezoidProfile.State(0.0, 0.0)
      );
      _elevatorFeedForward = new ElevatorFeedforward(
          ELEVATOR_CONSTS.control.ELEVATOR_KS_V,
          ELEVATOR_CONSTS.control.ELEVATOR_KG_V,
          ELEVATOR_CONSTS.control.ELEVATOR_KV_V_PER_MPS,
          ELEVATOR_CONSTS.control.ELEVATOR_KA_V_PER_MPSQ
      );

      if (Robot.isReal()) {
        int counter = 0;
        while (!checkInitStatus()) {
          System.out.println("ELEVATOR Check Init Status : " + counter++);
        }
      } else {
        simulationConfig();
      }

      configElevatorMotor();
      configFollowerMotor();

      configShuffleboard();

      LoopTimer.markEvent(" Elevator Initialization Complete: ");
    } else {
      _elevatorMotor = null;
      _followerMotor = null;
    }
  }

  public void initMotorConfigs() {
    /* Elevator Motor Configuration */
    _elevatorMotorFXConfig.slot0.kP = ElevatorConfs.ElevatorControl.ELEVATOR_PFAC;
    _elevatorMotorFXConfig.slot0.kI = ElevatorConfs.ElevatorControl.ELEVATOR_IFAC;
    _elevatorMotorFXConfig.slot0.kD = ElevatorConfs.ElevatorControl.ELEVATOR_DFAC;
    _elevatorMotorFXConfig.slot0.kF = ElevatorConfs.ElevatorControl.ELEVATOR_FFAC;
    _elevatorMotorFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        ElevatorConfs.ELEVATOR_SUPPLY_ENABLE_CURRENT_LIMIT,
        ElevatorConfs.ELEVATOR_SUPPLY_CONTINUOUS_CURRENT_LIMIT,
        ElevatorConfs.ELEVATOR_SUPPLY_PEAK_CURRENT_LIMIT,
        ElevatorConfs.ELEVATOR_SUPPLY_PEAK_CURRENT_DURATION
    );
    _elevatorMotorFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        ElevatorConfs.ELEVATOR_STATOR_ENABLE_CURRENT_LIMIT,
        ElevatorConfs.ELEVATOR_STATOR_CONTINUOUS_CURRENT_LIMIT,
        ElevatorConfs.ELEVATOR_STATOR_PEAK_CURRENT_LIMIT,
        ElevatorConfs.ELEVATOR_STATOR_PEAK_CURRENT_DURATION
    );
    _elevatorMotorFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    _elevatorMotorFXConfig.openloopRamp = ElevatorConfs.ElevatorControl.ELEVATOR_OPEN_LOOP_RAMP;
    _elevatorMotorFXConfig.closedloopRamp = ElevatorConfs.ElevatorControl.ELEVATOR_CLOSED_LOOP_RAMP;
    _elevatorMotorFXConfig.voltageCompSaturation =
        ElevatorConfs.ElevatorControl.ELEVATOR_VOLTAGE_COMP_SATURATION;
    _elevatorMotorFXConfig.neutralDeadband = ElevatorConfs.ELEVATOR_MOTOR_DEADBAND;
  }

  private void configElevatorMotor() {
    if (Constants.hasElevator) {
      _elevatorMotor.configFactoryDefault();
      _elevatorMotor.configAllSettings(_elevatorMotorFXConfig);
      _elevatorMotor.setInverted(ELEVATOR_CONSTS.ELEVATOR_MOTOR_INVERTED);
      _elevatorMotor.setNeutralMode(ELEVATOR_CONSTS.ELEVATOR_NEUTRAL_MODE);
      _elevatorMotor.setSelectedSensorPosition(0);
      _elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
  }

  private void configFollowerMotor() {
    if (Constants.hasElevator && Constants.isCompBot) {
      _followerMotor.configFactoryDefault();
      _followerMotor.configAllSettings(_elevatorMotorFXConfig);
      _followerMotor.setInverted(ELEVATOR_CONSTS.ELEVATOR_FOLLOWER_MOTOR_INVERTED);
      _followerMotor.setNeutralMode(ELEVATOR_CONSTS.ELEVATOR_NEUTRAL_MODE);
      _followerMotor.follow(_elevatorMotor);
    }
  }

  public void configShuffleboard() {
    if (Constants.hasElevator) {
      ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");
      if (RobotAltModes.isVerboseMode) {
        ShuffleboardLayout elevatorLayout =
            tuningTab.getLayout("Elevator", BuiltInLayouts.kGrid).withSize(1, 4).withPosition(0, 0);

        _currentTicksEntry =
            elevatorLayout.add("Current Position", 0.0).withPosition(0, 0).getEntry();
        _targetTicksEntry =
            elevatorLayout.add("Target Position", 0.0).withPosition(0, 1).getEntry();

        _elevatorPercentEntry =
            elevatorLayout.add("Percent output", 0.0).withPosition(0, 2).getEntry();

        _elevatorCurrentVelocityEntry =
            elevatorLayout.add("Current Velocity", 0.0).withPosition(0, 3).getEntry();
        _elevatorTargetVelocityEntry =
            elevatorLayout.add("Target Velocity", 0.0).withPosition(0, 4).getEntry();
        _elevatorProfilePositionEntry =
            elevatorLayout.add("Profile Position", 0.0).withPosition(0, 5).getEntry();

        _elevatorAccelEntry =
            elevatorLayout.add("Current Accelerations", 0.0).withPosition(0, 5).getEntry();

        _statorCurrentEntry =
            elevatorLayout.add("Current Stator Current", 0.0).withPosition(0, 6).getEntry();
        _supplyCurrentEntry =
            elevatorLayout.add("Current Supply Current", 0.0).withPosition(0, 7).getEntry();
      }
      if (RobotAltModes.isElevatorTuningMode) {
        _kGEntry = tuningTab.add("kG", ELEVATOR_CONSTS.control.ELEVATOR_KG_V)
                       .withPosition(2, 0)
                       .getEntry();
        _kSEntry = tuningTab.add("kS", ELEVATOR_CONSTS.control.ELEVATOR_KS_V)
                       .withPosition(2, 1)
                       .getEntry();
        _kVEntry = tuningTab.add("kV", ELEVATOR_CONSTS.control.ELEVATOR_KV_V_PER_MPS)
                       .withPosition(2, 2)
                       .getEntry();
        _kAEntry = tuningTab.add("kA", ELEVATOR_CONSTS.control.ELEVATOR_KA_V_PER_MPSQ)
                       .withPosition(2, 3)
                       .getEntry();
        _elevatorPowerEntry = tuningTab.add("Power", 0.0).withPosition(1, 3).getEntry();
        _elevatorPositionErrorEntry =
            tuningTab.add("Elevator Pos Error", 0.0).withPosition(0, 4).getEntry();
        _elevatorVelocityErrorEntry =
            tuningTab.add("Elevator Velo Error", 0.0).withPosition(0, 5).getEntry();
        _maxVeloEntry =
            tuningTab.add("Elevator Max Velo", ELEVATOR_CONSTS.control.ELEVATOR_MAX_VELO_MPS)
                .withPosition(10, 0)
                .getEntry();
        _maxAccelEntry =
            tuningTab.add("Elevator Max Accel", ELEVATOR_CONSTS.control.ELEVATOR_MAX_ACC_MPSQ)
                .withPosition(10, 1)
                .getEntry();
      }
      if (RobotAltModes.isPIDTuningMode || RobotAltModes.isElevatorTuningMode) {
        _pFactorEntry = tuningTab.add("Height P Fac", ElevatorConfs.ElevatorControl.ELEVATOR_PFAC)
                            .withPosition(3, 0)
                            .getEntry();
        _iFactorEntry = tuningTab.add("Height I Fac", ElevatorConfs.ElevatorControl.ELEVATOR_IFAC)
                            .withPosition(4, 0)
                            .getEntry();
        _dFactorEntry = tuningTab.add("Height D Fac", ElevatorConfs.ElevatorControl.ELEVATOR_DFAC)
                            .withPosition(5, 0)
                            .getEntry();
      }
    }
  }

  public void updateShuffleboard() {
    if (Constants.hasElevator && RobotAltModes.isElevatorTuningMode) {
      _currentTicksEntry.setInteger(getCurrentHeight());
      _targetTicksEntry.setInteger(getTargetHeight());
      _elevatorPositionErrorEntry.setInteger(getTargetHeight() - getCurrentHeight());
      _elevatorPercentEntry.setDouble(getPower());

      _statorCurrentEntry.setDouble(_elevatorMotor.getStatorCurrent());
      _supplyCurrentEntry.setDouble(_elevatorMotor.getSupplyCurrent());
    }
  }

  @Override
  public void periodic() {
    if (Constants.hasElevator) {
      double currentVelo = CTREConversion.falconToMPS(
          _elevatorMotor.getSelectedSensorVelocity(),
          ElevatorConfs.ElevatorDims.ELEVATOR_CIRCUMFERENCE_M,
          ElevatorConfs.ElevatorDims.ELEVATOR_GEAR_RATIO
      );

      if (RobotAltModes.isElevatorTuningMode) {
        _elevatorCurrentVelocityEntry.setDouble(currentVelo);

        _elevatorAccelEntry.setDouble(currentVelo - _prevVelo);
        _prevVelo = currentVelo;
      }
      LoopTimer.markLoopStart();
      if (_followingProfile) {
        State goalState = _profile.calculate(_timer.get());
        double goalVelo = goalState.velocity;

        if (RobotAltModes.isElevatorTuningMode) {
          _elevatorTargetVelocityEntry.setDouble(goalState.velocity);

          _elevatorVelocityErrorEntry.setDouble(goalVelo - currentVelo);
        }

        double desiredAccelMPS2 =
            (goalState.velocity - _prevVelocity_MPS) / Constants.LOOP_PERIOD_S;
        _prevVelocity_MPS = goalState.velocity;

        double ff = _elevatorFeedForward.calculate(goalState.velocity, desiredAccelMPS2)
            / ElevatorControl.ELEVATOR_VOLTAGE_COMP_SATURATION;

        _elevatorMotor.set(
            ControlMode.Position,
            // TODO check if this is supposed to be _lastExpectedHeight
            CTREConversion.metersToFalcon(
                goalState.position,
                ElevatorConfs.ElevatorDims.ELEVATOR_CIRCUMFERENCE_M,
                ElevatorConfs.ElevatorDims.ELEVATOR_GEAR_RATIO
            ),
            DemandType.ArbitraryFeedForward,
            ff
        );
        _elevatorProfilePositionEntry.setDouble(CTREConversion.metersToFalcon(
            goalState.position,
            ElevatorConfs.ElevatorDims.ELEVATOR_CIRCUMFERENCE_M,
            ElevatorConfs.ElevatorDims.ELEVATOR_GEAR_RATIO
        ));
      }

      updateShuffleboard();
      LoopTimer.markCompletion(" Elevator Profile Following ", "\n Total Elevator: ");
    }
  }

  public void setPower(double power) {
    if (Constants.hasElevator) {
      _followingProfile = false;
      _elevatorMotor.set(power);
    }
  }

  public double getPower() {
    if (Constants.hasElevator) {
      return _elevatorMotor.getMotorOutputPercent();
    }
    return 0.0;
  }

  public boolean isClearOfIntake() {
    if (Constants.hasElevator) {
      return getCurrentHeight() > ElevatorConfs.ELEVATOR_CLEAR_OF_INTAKE_TICKS;
    }
    return true;
  }

  public int getCurrentHeight() {
    if (Constants.hasElevator) {
      return (int) _elevatorMotor.getSelectedSensorPosition();
    }
    return _clampedPosition;
  }

  public int getTargetHeight() {
    return _clampedPosition;
  }

  public void setTargetHeight(ElevatorPositions target) {
    _followingProfile = false;
    _goalPosition = target;
    _clampedPosition = clamp(target.ticks);
    if (Constants.hasElevator) {
      _elevatorMotor.set(ControlMode.Position, _clampedPosition);
    }
  }

  public double getCurrentVelocity() {
    if (Constants.hasElevator) {
      return _elevatorMotor.getSelectedSensorVelocity();
    }
    return 0.0;
  }

  public boolean inTolerance(int lowerTolerance, int upperTolerance) {
    if (Constants.hasElevator) {
      int error = getCurrentHeight() - getTargetHeight();
      return error >= lowerTolerance && error <= upperTolerance;
    }
    return true;
  }

  public void setEncoder(int ticks) {
    if (Constants.hasElevator) {
      _elevatorMotor.setSelectedSensorPosition(ticks);
      _goalPosition = ElevatorPositions.ELEVATOR_MIN_TICKS;
      _clampedPosition = ticks;
      _followingProfile = false;
    }
  }

  public boolean motorResetConfig() {
    if (Constants.hasElevator) {
      if (_elevatorMotor.hasResetOccurred()) {
        configElevatorMotor();
        return true;
      }
      if (Constants.isCompBot && _followerMotor.hasResetOccurred()) {
        configFollowerMotor();
        return true;
      }
    }
    return false;
  }

  // we should just map these to levels
  public ElevatorPositions getElevatorScoringHeight(GamePieceType heldPiece, ScoringLevels level) {
    if (heldPiece.isUnknown() || level == ScoringLevels.UNKNOWN_SCORING_LEVEL) {
      return _goalPosition;
    }
    return level.getPosition(heldPiece);
  }

  public void onEnable() {
    if (Constants.hasElevator) {
      _followingProfile = false;
      _elevatorMotor.setNeutralMode(ElevatorConfs.ELEVATOR_NEUTRAL_MODE);
      if (Constants.isCompBot) {
        _followerMotor.setNeutralMode(ElevatorConfs.ELEVATOR_NEUTRAL_MODE);
      }
      if (RobotAltModes.isPIDTuningMode || RobotAltModes.isElevatorTuningMode) {
        _elevatorMotor.config_kP(
            0, _pFactorEntry.getDouble(ElevatorConfs.ElevatorControl.ELEVATOR_PFAC)
        );
        _elevatorMotor.config_kI(
            0, _iFactorEntry.getDouble(ElevatorConfs.ElevatorControl.ELEVATOR_IFAC)
        );
        _elevatorMotor.config_kD(
            0, _dFactorEntry.getDouble(ElevatorConfs.ElevatorControl.ELEVATOR_DFAC)
        );
      }
      if (RobotAltModes.isElevatorTuningMode) {
        _elevatorFeedForward = new ElevatorFeedforward(
            _kSEntry.getDouble(ELEVATOR_CONSTS.control.ELEVATOR_KS_V),
            _kGEntry.getDouble(ELEVATOR_CONSTS.control.ELEVATOR_KG_V),
            _kVEntry.getDouble(ELEVATOR_CONSTS.control.ELEVATOR_KV_V_PER_MPS),
            _kAEntry.getDouble(ELEVATOR_CONSTS.control.ELEVATOR_KA_V_PER_MPSQ)
        );
      }
    }
  }

  public void onDisable() {
    if (Constants.hasElevator) {
      _followingProfile = false;
      _elevatorMotor.setNeutralMode(ElevatorConfs.ELEVATOR_DISABLED_MODE);
      if (Constants.isCompBot) {
        _followerMotor.setNeutralMode(ElevatorConfs.ELEVATOR_DISABLED_MODE);
      }
      _profile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ELEVATOR_CONSTS.control.ELEVATOR_MAX_VELO_MPS,
              ELEVATOR_CONSTS.control.ELEVATOR_MAX_ACC_MPSQ
          ),
          new TrapezoidProfile.State(0, 0)
      );
    }
  }

  public void setPositionalArbitraryFF(double position, double ff) {
    if (Constants.hasElevator) {
      _followingProfile = false;
      _elevatorMotor.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, ff);
    }
  }

  public void setElevatorProfile(ElevatorPositions position) {
    setElevatorProfile(position, 0.0);
  }

  public void setElevatorProfile(ElevatorPositions position, double velocity) {
    setElevatorProfile(
        position, velocity, _maxVeloEntry.getDouble(ELEVATOR_CONSTS.control.ELEVATOR_MAX_VELO_MPS)
    );
  }

  public void setElevatorProfile(ElevatorPositions position, double velocity, double maxV) {
    setElevatorProfile(
        position,
        velocity,
        maxV,
        _maxAccelEntry.getDouble(ELEVATOR_CONSTS.control.ELEVATOR_MAX_ACC_MPSQ)
    );
  }

  public void setElevatorProfile(
      ElevatorPositions goalPosition, double goalVelocityMPS, double maxV, double maxA
  ) {
    if (Constants.hasElevator) {
      _followingProfile = true;
      _goalPosition = goalPosition;
      _clampedPosition = clamp(goalPosition.ticks);
      TrapezoidProfile.State currentState = _profile.calculate(_timer.get());
      currentState.position = CTREConversion.falconToMeters(
          _elevatorMotor.getSelectedSensorPosition(),
          ElevatorConfs.ElevatorDims.ELEVATOR_CIRCUMFERENCE_M,
          ElevatorConfs.ElevatorDims.ELEVATOR_GEAR_RATIO
      );
      _profile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(maxV, maxA),
          new TrapezoidProfile.State(
              CTREConversion.falconToMeters(
                  _clampedPosition,
                  ElevatorConfs.ElevatorDims.ELEVATOR_CIRCUMFERENCE_M,
                  ElevatorConfs.ElevatorDims.ELEVATOR_GEAR_RATIO
              ),
              goalVelocityMPS
          ),
          currentState
      );

      _prevVelocity_MPS = currentState.velocity;
      _timer.restart();
    }
  }

  private boolean checkInitStatus() {
    if (Constants.hasElevator) {
      ErrorCode initStatus = _elevatorMotor.configFactoryDefault();
      return initStatus == ErrorCode.OK;
    }
    return true;
  }

  private int clamp(int target, int min, int max) {
    return Math.max(min, Math.min(max, target));
  }

  private int clamp(int target) {
    return clamp(
        target,
        ElevatorPositions.ELEVATOR_MIN_FALCON_TICKS,
        ElevatorPositions.ELEVATOR_MAX_FALCON_TICKS
    );
  }

  public void simulationConfig() {
    if (Constants.hasElevator && RobotAltModes.isSim) {
      if (RobotAltModes.isSimElevator) {
        /* TODO: fill in with InclinedSimElevator objects */
      }
      _mech2d = new Mechanism2d(80, 80);
      _elevatorRaise = _mech2d.getRoot("ElevatorRaise", 30, 30);
      _elevatorTower = _elevatorRaise.append(
          new MechanismLigament2d("BaseStage", 30, 225, 6, new Color8Bit(Color.kBlue))
      );
      _elevatorStage2 = _elevatorRaise.append(
          new MechanismLigament2d("Stage1", 30, 45, 6, new Color8Bit(Color.kLightPink))
      );

      SmartDashboard.putData("Elevator Sim", _mech2d);
    }
  }
}
