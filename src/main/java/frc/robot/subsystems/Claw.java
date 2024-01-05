package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.utils.LoopTimer;

public class Claw extends SubsystemBase {
  /* --- Motors, sensors, and hardware variables --- */
  private DigitalInput _clawBeamBreak;

  private WPI_TalonSRX _primaryClawRollerMotor;
  private WPI_TalonSRX _leftClawRollerMotor;
  private DoubleSolenoid _clawSolenoid;

  // Motorized Wrist Klaw
  // private WPI_TalonFX _wristMotor;
  // private WPI_TalonFX _clawRollerMotor;
  // private CANCoder _clawCancoder;

  /* --- Configuration Sets --- */
  private TalonSRXConfiguration _rollerMotorsConfig;
  private TalonFXConfiguration _wristMotorConfig;

  /* --- State Variables --- */
  private ClawPositions _goalPosition = ClawPositions.CLAW_POSITION_UNKNOWN;

  /* --- Shuffleboard Entries --- */
  private GenericEntry _clawRollersEntry;
  private GenericEntry _beamBreakEntry;

  public Claw() {
    if (Constants.hasClaw) {
      initRollerMotorConfig();
      initWristMotorConifg();

      _clawBeamBreak = new DigitalInput(Constants.CRobot.claw.ports.breakBeam);

      if (Constants.isKlaw || Constants.isPneumaticClaw) {
        _clawSolenoid = new DoubleSolenoid(
            Constants.CRobot.air.moduleID,
            Constants.CRobot.air.moduleType,
            Constants.CRobot.air.clawForward,
            Constants.CRobot.air.clawReverse
        );
        _primaryClawRollerMotor = new WPI_TalonSRX(Constants.CRobot.claw.ports.leftRoller);
        _leftClawRollerMotor = new WPI_TalonSRX(Constants.CRobot.claw.ports.rightRoller);
      }
      // else {
      // _wristMotor = new WPI_TalonFX(Constants.CRobot.claw.ports.);
      // _clawCancoder = new CANCoder(ClawPorts.CLAW_CANCODER_ID);
      // }

      waitForCAN();

      configClawMotor();
      configLeftClawRollerMotor();
      configRightClawRollerMotor();
    }
    configShuffleboard();
    LoopTimer.markEvent(" Claw Initialization Complete: ");
  }

  @Override
  public void periodic() {
    _beamBreakEntry.setBoolean(getBeamBreak());
  }

  public void disabledPeriodic() {}

  public void setClawPower(double speed) {}

  public int getTargetPosition() {
    return _goalPosition.ticks;
  }

  public void setTargetPosition(ClawPositions pos) {
    if (pos.ticks < ClawPositions.CLAW_OPEN_POS_TICKS.ticks) {
      closeClawSolenoid();
    } else {
      openClawSolenoid();
    }
  }

  public double getClawPower() {
    return 0.0;
  }

  public int getClawPosition() {
    return _goalPosition.ticks;
  }

  public void closeClawSolenoid() {
    if (Constants.hasClaw && Constants.isPneumaticClaw) {
      _goalPosition = ClawPositions.CLAW_MIN_TICKS;
      _clawSolenoid.set(ClawConfs.CLAW_SOLENOID_CLOSED);
    }
  } // close arm

  public void openClawSolenoid() {
    if (Constants.hasClaw && Constants.isPneumaticClaw) {
      _goalPosition = ClawPositions.CLAW_OPEN_POS_TICKS;
      _clawSolenoid.set(ClawConfs.CLAW_SOLENOID_OPEN);
    }
  } // extend arm

  public void setLeftClawRollerPower(double speed) {
    if (Constants.hasClaw) {
      _primaryClawRollerMotor.set(speed);
    }
  }

  public double getLeftClawRollerPower() {
    if (Constants.hasClaw) {
      return _primaryClawRollerMotor.getMotorOutputPercent();
    }
    return 0.0;
  }

  public void setRightClawRollerPower(double speed) {
    if (Constants.hasClaw) {
      _leftClawRollerMotor.set(speed);
    }
  }

  public double getRightClawRollerPower() {
    if (Constants.hasClaw) {
      return _leftClawRollerMotor.getMotorOutputPercent();
    }
    return 0.0;
  }

  public void setClawRollersPower(double speed) {
    if (Constants.hasClaw) {
      _primaryClawRollerMotor.set(speed);
      _leftClawRollerMotor.set(speed);
    }
  }

  public void setClawRollersStall(GamePieceType piece) {
    setClawPower(
        piece.isCube() ? -ClawConfs.CLAW_CUBE_STALL_POWER : ClawConfs.CLAW_CONE_STALL_POWER
    );
  }

  public double getClawRollersPower() {
    if (Constants.hasClaw) {
      return _leftClawRollerMotor.getMotorOutputPercent();
    }
    return 0.0;
  }

  public boolean getBeamBreak() {
    if (Constants.hasClaw) {
      return !_clawBeamBreak.get();
    }
    return true;
  }

  public boolean coneDetected(double threshold) {
    if (Constants.hasClaw) {
      return threshold <= _leftClawRollerMotor.getSupplyCurrent()
          || threshold <= _primaryClawRollerMotor.getSupplyCurrent();
    }
    return true;
  }

  public boolean coneDetected() {
    if (Constants.hasClaw) {
      return coneDetected(ClawConfs.IntakeSpeed.CONE_DETECTION_CURRENT_THRESHOLD_A);
    }
    return true;
  }

  public boolean inTolerance(int lowerBound, int upperBound) {
    return true;
  }

  public boolean inTolerance(int tolerance) {
    return true;
  }

  public boolean motorResetConfig() {
    if (Constants.hasClaw) {
      if (_leftClawRollerMotor.hasResetOccurred()) {
        configRightClawRollerMotor();
        return true;
      }
      if (_primaryClawRollerMotor.hasResetOccurred()) {
        configLeftClawRollerMotor();
        return true;
      }
    }
    return false;
  }

  public boolean isExtended() {
    return getClawPosition() >= ClawPositions.CLAW_OPEN_POS_THRESHOLD.ticks;
  }

  public boolean isRetracted() {
    return getClawPosition() <= ClawPositions.CLAW_CLOSED_POS_THRESHOLD.ticks;
  }

  /* --- Initialization Functions --- */
  private boolean checkInitStatus() {
    if (Constants.hasClaw) {
      ErrorCode initStatus = _primaryClawRollerMotor.configFactoryDefault();
      return initStatus == ErrorCode.OK;
    }
    return true;
  }

  private void waitForCAN() {
    if (Constants.hasClaw && Robot.isReal()) {
      int counter = 0;
      while (!checkInitStatus()) {
        System.out.println("CLAW Check Init Status : " + counter++);
      }
    }
  }

  private void configShuffleboard() {
    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");

    _beamBreakEntry = tuningTab.add("Break Beam", getBeamBreak()).withPosition(1, 0).getEntry();
  }

  private void initRollerMotorConfig() {
    if (Constants.hasClaw) {
      _rollerMotorsConfig = new TalonSRXConfiguration();
      // _rollerMotorsConfig.slot0.kP               = CLAW_ROLLERS_PFAC;
      // _rollerMotorsConfig.slot0.kI               = CLAW_ROLLERS_IFAC;
      // _rollerMotorsConfig.slot0.kD               = CLAW_ROLLERS_DFAC;
      // _rollerMotorsConfig.slot0.kF               = CLAW_ROLLERS_FFAC;
      // _rollerMotorsConfig.voltageCompSaturation = 12.0;
      _rollerMotorsConfig.openloopRamp = ClawConfs.CLAW_OPEN_LOOP_RAMP;
      _rollerMotorsConfig.closedloopRamp = ClawConfs.CLAW_CLOSED_LOOP_RAMP;
      _rollerMotorsConfig.peakCurrentLimit = ClawConfs.CLAW_ROLLERS_SUPPLY_PEAK_CURRENT_LIMIT;
      _rollerMotorsConfig.peakCurrentDuration =
          ClawConfs.CLAW_ROLLERS_SUPPLY_PEAK_CURRENT_DURATION_MS;
      _rollerMotorsConfig.continuousCurrentLimit =
          ClawConfs.CLAW_ROLLERS_SUPPLY_CONTINUOUS_CURRENT_LIMIT;
    }
  }

  private void initWristMotorConifg() {
    if (Constants.hasClaw) {
      _wristMotorConfig = new TalonFXConfiguration();
    }
  }

  private void configClawMotor() {
    // _wristMotor.configFactoryDefault();
    // TODO
  }

  private void configLeftClawRollerMotor() {
    if (Constants.hasClaw) {
      _leftClawRollerMotor.configFactoryDefault();
      _leftClawRollerMotor.configAllSettings(_rollerMotorsConfig);
      _leftClawRollerMotor.setInverted(ClawConfs.LEFT_ROLLERS_INVERTED);
      _leftClawRollerMotor.setNeutralMode(ClawConfs.ROLLER_NEUTRAL_MODE);
      _leftClawRollerMotor.follow(_primaryClawRollerMotor);
    }
  }

  private void configRightClawRollerMotor() {
    if (Constants.hasClaw) {
      _primaryClawRollerMotor.configFactoryDefault();
      _primaryClawRollerMotor.configAllSettings(_rollerMotorsConfig);
      _primaryClawRollerMotor.setInverted(ClawConfs.PRIMARY_ROLLERS_INVERTED);
      _primaryClawRollerMotor.setNeutralMode(ClawConfs.ROLLER_NEUTRAL_MODE);
    }
  }

  /* --- Basic Instant Commands --- */
  public CommandBase closeClawCommand() {
    return closeClawCommand(false);
  }
  public CommandBase closeClawCommand(boolean require) {
    return require ? runOnce(() -> closeClawSolenoid())
                   : Commands.runOnce(() -> closeClawSolenoid());
  }

  public CommandBase openClawCommand() {
    return openClawCommand(false);
  }
  public CommandBase openClawCommand(boolean require) {
    return require ? runOnce(() -> openClawSolenoid()) : Commands.runOnce(() -> openClawSolenoid());
  }

  public CommandBase setClawPowerCommand(double speed) {
    return setClawPowerCommand(speed, false);
  }
  public CommandBase setClawPowerCommand(double speed, boolean require) {
    return require ? runOnce(() -> setClawPower(speed))
                   : Commands.runOnce(() -> setClawPower(speed));
  }

  public CommandBase setClawRollersPowerCommand(double speed) {
    return runOnce(() -> setClawRollersPower(speed));
  }

  public CommandBase clawRollersOffCommand() {
    return runOnce(() -> setClawRollersPower(0.0));
  }
}
