package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.LedConfs;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.HPIntakeStations;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.LedModes;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.constants.enums.ScoringLocations;
import frc.robot.utils.LoopTimer;
import java.util.LinkedList;

public class LedController extends SubsystemBase {
  /* --- Hardware and Configuration --- */
  private CANdle _candle;
  private CANdleConfiguration _candleConfigs;

  /* --- State and Physical Property variables --- */
  private LedModes _curMode = LedModes.COLOR_FLOW;
  private ScoringLocations _tempMode = ScoringLocations.SCORING_LOCATION_1;
  private HPIntakeStations _hpMode = HPIntakeStations.UNKNOWN_INTAKE_STATION;
  private boolean _disabled;
  private boolean _lastVision = false;
  private boolean _isOverriden = false;

  /* --- Control Utils --- */
  /* Generic Animations */
  LinkedList<LedColors> colorsQueue = new LinkedList<LedColors>();
  LinkedList<LedSections> sectionsQueue = new LinkedList<LedSections>();

  public LedController() {
    if (Constants.hasLed) {
      _candle =
          new CANdle(Constants.CRobot.led.ports.candle, Constants.CRobot.led.ports.candleCanBus);
      _candleConfigs = new CANdleConfiguration();
      _candleConfigs.statusLedOffWhenActive = true;
      _candleConfigs.disableWhenLOS = false;
      _candleConfigs.stripType = LEDStripType.GRB;
      _candleConfigs.brightnessScalar = 0.1;
      _candleConfigs.vBatOutputMode = VBatOutputMode.Modulated;

      if (Robot.isReal()) {
        int counter = 0;
        while (!checkInitStatus()) {
          System.out.println("CANDLE Check Init Status : " + counter++);
        }
      } else {
      }

      configCandle();
      _candle.animate(LedModes.RAINBOW.animation);
    }
    LoopTimer.markEvent(" LED Initialization Complete: ");
  }

  @Override
  public void periodic() {
    LoopTimer.markLoopStart();

    // colorsQueue, sectionsQueue
    if (colorsQueue.size() > 0 && sectionsQueue.size() > 0 && !_disabled) {
      LedColors desiredLEDColor = colorsQueue.getFirst();
      LedSections desiredLEDSection = sectionsQueue.getFirst();

      colorsQueue.removeFirst();
      sectionsQueue.removeFirst();
      setSolidColor(desiredLEDColor, desiredLEDSection);
    }

    LoopTimer.markCompletion("\n Total LED Controller: ");
  }

  public boolean checkInitStatus() {
    return Constants.isCompBot && _candle.configFactoryDefault() == ErrorCode.OK;
  }

  public void configCandle() {
    if (Constants.hasLed) {
      _candle.configFactoryDefault();
      _candle.configAllSettings(_candleConfigs, 100);
    }
  }

  public void disabledPeriodic(Pose2d currPose, Pose2d desiredPose, Boolean blueAlliance) {
    Transform2d transform = new Transform2d(currPose, desiredPose);
    double transformX = transform.getX();
    boolean xCondition = Math.abs(transformX) <= Constants.CRobot.drive.control.xy.toleranceM * 3;
    double transformY = transform.getY();
    boolean yCondition = Math.abs(transformY) <= Constants.CRobot.drive.control.xy.toleranceM;
    boolean angleCondition = Math.abs(transform.getRotation().getRadians())
        <= Constants.CRobot.drive.control.theta.tolerance.getRadians() * 3;

    if (xCondition && yCondition && angleCondition) {
      setSolidColor(LedColors.GREEN, LedSections.CANDLE);
      setSolidColor(LedColors.GREEN, LedSections.STRIP);
      setSolidColor(LedColors.GREEN, LedSections.STRIP_2);
      return;
    }

    if (xCondition) {
      setSolidColor(LedColors.SPACE_COOKIE, LedSections.STRIP);
      setSolidColor(LedColors.SPACE_COOKIE, LedSections.STRIP_2);
    } else {
      int ledChange = Math.max(0, Math.min(7, (int) Math.abs(transformX)));
      if (transformX > 0) {
        setSolidColor(
            LedColors.OFF,
            LedConfs.STRIP_1_START,
            LedConfs.STRIP_1_START + LedConfs.STRIP_LENGTH - ledChange
        );
        setSolidColor(
            LedColors.BLUE,
            LedConfs.STRIP_1_START + LedConfs.STRIP_LENGTH - ledChange,
            LedConfs.STRIP_1_START + LedConfs.STRIP_LENGTH
        );
        setSolidColor(
            LedColors.OFF,
            LedConfs.STRIP_2_START + ledChange,
            LedConfs.STRIP_2_START + LedConfs.STRIP_LENGTH
        );
        setSolidColor(LedColors.BLUE, LedConfs.STRIP_2_START, LedConfs.STRIP_2_START + ledChange);
      } else {
        setSolidColor(
            LedColors.OFF,
            LedConfs.STRIP_1_START + ledChange,
            LedConfs.STRIP_1_START + LedConfs.STRIP_LENGTH
        );
        setSolidColor(LedColors.PURPLE, LedConfs.STRIP_1_START, LedConfs.STRIP_1_START + ledChange);
        setSolidColor(
            LedColors.OFF,
            LedConfs.STRIP_2_START,
            LedConfs.STRIP_2_START + LedConfs.STRIP_LENGTH - ledChange
        );
        setSolidColor(LedColors.PURPLE, LedConfs.LED_LENGTH - ledChange, LedConfs.LED_LENGTH);
      }
    }

    if (yCondition) {
      setSolidColor(LedColors.SPACE_COOKIE, LedSections.INDICATOR);
    } else {
      if (transformY > 0) {
        setSolidColor(LedColors.BLUE, LedSections.INDICATOR_HALF_2);
        turnOffLeds(LedSections.INDICATOR_HALF_1);
      } else {
        setSolidColor(LedColors.BLUE, LedSections.INDICATOR_HALF_1);
        turnOffLeds(LedSections.INDICATOR_HALF_2);
      }
    }
  }

  public void changeAnimation(LedModes desiredAnimation) {
    if (Constants.hasLed) {
      _candle.animate(desiredAnimation.animation);
      _curMode = desiredAnimation;
    }
  }

  // testing
  public void incrementAnimation() {
    _tempMode = _tempMode.getIncr();
    // _hpMode = _hpMode.getIncr();
    // changeAnimation(_curMode);
    // setIntakeMode(_hpMode, GamePieceType.CUBE_GAME_PIECE);
    setScoringMode(ScoringLevels.SCORING_LEVEL_2, _tempMode);
  }
  // testing
  public void decrementAnimation() {
    _curMode = _curMode.getDecr();
    changeAnimation(_curMode);
  }

  public void setSolidColor(int r, int g, int b, int w, int start, int end) {
    if (Constants.hasLed) {
      _candle.clearAnimation(0);
      _candle.setLEDs(r, g, b, w, start, end - start);
    }
  }

  public void setSolidColor(int r, int g, int b, int w, LedSections ledSection) {
    setSolidColor(r, g, b, w, ledSection.start, ledSection.end);
  }

  public void setSolidColor(LedColors ledColor, int start, int end) {
    setSolidColor(ledColor.r, ledColor.g, ledColor.b, LedConfs.LED_WHITE_LEVEL, start, end);
  }

  public void setSolidColor(LedColors ledColor, LedConfs.LedSections ledSection) {
    setSolidColor(ledColor.r, ledColor.g, ledColor.b, LedConfs.LED_WHITE_LEVEL, ledSection);
  }

  public void setScoringColor(ScoringLevels scoringLevel, ScoringLocations scoringLocation) {
    setScoringColor(scoringLevel, scoringLocation.section, scoringLocation.subsection);
  }

  public CommandBase setSolidColorCommand(LedColors ledColor, LedConfs.LedSections ledSection) {
    return Commands.runOnce(() -> setSolidColor(ledColor, ledSection));
  }

  public void setScoringColor(
      ScoringLevels scoringLevel,
      LedConfs.ScoringSections scoringSection,
      LedConfs.ScoringSubsections scoringSubsection
  ) {
    // pushQueueCounter(LedColors.OFF, LedSections.STRIP);
    // pushQueueCounter(LedColors.OFF, LedSections.STRIP_2);

    pushQueueCounter(scoringSection.sectionColor, LedSections.STRIP_HALF_1);
    pushQueueCounter(scoringSubsection.subsectionColor, LedSections.STRIP_HALF_2);
    pushQueueCounter(scoringSection.sectionColor, LedSections.STRIP_2_HALF_1);
    pushQueueCounter(scoringSubsection.subsectionColor, LedSections.STRIP_2_HALF_2);

    pushQueueCounter(LedColors.OFF, LedSections.INDICATOR);

    switch (scoringLevel) {
      case SCORING_LEVEL_0:
        pushQueueCounter(LedColors.GREEN, LedSections.INDICATOR_THIRD_1);
        // pushQueueCounter(LedColors.GREEN, LedSections.STRIP_THIRD_1);
        // pushQueueCounter(LedColors.GREEN, LedSections.STRIP_2_THIRD_1);

        // pushQueueCounter(LedColors.OFF, LedSections.STRIP);
        // pushQueueCounter(LedColors.sectionColor, LedSections.STRIP_THIRD_1);
        // pushQueueCounter(LedColors.subsectionColor, LedSections.STRIP_THIRD_2);
        break;
      case SCORING_LEVEL_1:
        pushQueueCounter(LedColors.BLUE, LedSections.INDICATOR_THIRD_1);
        pushQueueCounter(LedColors.BLUE, LedSections.INDICATOR_THIRD_2);

        // pushQueueCounter(LedColors.OFF, LedSections.STRIP);
        // pushQueueCounter(LedColors.OFF, LedSections.STRIP_2);
        // pushQueueCounter(LedColors.BLUE, LedSections.STRIP_THIRD_1);
        // pushQueueCounter(LedColors.BLUE, LedSections.STRIP_2_THIRD_1);
        // pushQueueCounter(LedColors.BLUE, LedSections.STRIP_THIRD_2);
        // pushQueueCounter(LedColors.BLUE, LedSections.STRIP_2_THIRD_2);
        // pushQueueCounter(LedColors.OFF, LedSections.STRIP);
        // pushQueueCounter(LedColors.sectionColor, LedSections.STRIP_THIRD_1);
        // pushQueueCounter(LedColors.subsectionColor, LedSections.STRIP_THIRD_2);
        // pushQueueCounter(LedColors.sectionColor, LedSections.STRIP_THIRD_3);
        // pushQueueCounter(LedColors.subsectionColor, LedSections.STRIP_THIRD_4);
        break;
      case SCORING_LEVEL_2:
        pushQueueCounter(LedColors.PINK, LedSections.INDICATOR);
        // pushQueueCounter(LedColors.OFF, LedSections.STRIP);
        // pushQueueCounter(LedColors.OFF, LedSections.STRIP_2);
        // pushQueueCounter(LedColors.PINK, LedSections.STRIP);
        // pushQueueCounter(LedColors.PINK, LedSections.STRIP_2);
        // pushQueueCounter(LedColors.OFF, LedSections.STRIP);
        // pushQueueCounter(LedColors.sectionColor, LedSections.STRIP);
        // pushQueueCounter(LedColors.subsectionColor, LedSections.STRIP_THIRD_2);
        // pushQueueCounter(LedColors.subsectionColor, LedSections.STRIP_THIRD_4);
        // pushQueueCounter(LedColors.subsectionColor, LedSections.STRIP_THIRD_6);
        break;
      default:
        pushQueueCounter(LedColors.OFF, LedSections.INDICATOR);
        // pushQueueCounter(LedColors.OFF, LedSections.STRIP));
        // pushQueueCounter(LedColors.OFF, LedSections.STRIP_2);
        break;
    }
    pushQueueCounter(LedColors.BLUE, LedSections.CANDLE);
  }

  public void setScoringMode(ScoringLevels scoringLevel, ScoringLocations scoringLocation) {
    if (scoringLocation == ScoringLocations.UNKNOWN_SCORING_LOCATION) {
      turnOffLeds(LedSections.NON_CANDLE);
    } else {
      setScoringColor(scoringLevel, scoringLocation.section, scoringLocation.subsection);
    }
  }

  public void turnOffLeds(int start, int end) {
    setSolidColor(0, 0, 0, 0, start, end);
  }

  public void turnOffLeds(LedSections ledSection) {
    setSolidColor(LedColors.OFF, ledSection);
  }

  public void setIntakeMode(HPIntakeStations HPIntakeStation, GamePieceType gamePieceType) {
    pushQueueCounter(gamePieceType.intakeColor, LedSections.INDICATOR_HALF_2);
    pushQueueCounter(gamePieceType.intakeColor, LedSections.STRIP_2);

    pushQueueCounter(HPIntakeStation.stationColor, LedSections.INDICATOR_HALF_1);
    pushQueueCounter(HPIntakeStation.stationColor, LedSections.STRIP);

    pushQueueCounter(LedColors.RED_ORANGE, LedSections.CANDLE);
  }

  public boolean ledResetConfig() {
    if (Constants.hasLed && _candle.hasResetOccurred()) {
      configCandle();
      return true;
    }
    return false;
  }

  public void pushQueueCounter(LedColors color, LedConfs.LedSections section) {
    colorsQueue.addLast(color);
    sectionsQueue.addLast(section);
  }

  public void setDisabled(Boolean disabled) {
    _disabled = disabled;
  }

  public void setVisionLed(Boolean hasVision) {
    if (hasVision != _lastVision) {
      pushQueueCounter(hasVision ? LedColors.GREEN : LedColors.YELLOW, LedSections.CANDLE);
    }
    _lastVision = hasVision;
  }

  public void setOverride(Boolean override) {
    _isOverriden = override;

    // Clear any accumulated teleop LED queue
    if (_isOverriden == false) {
      colorsQueue.clear();
      sectionsQueue.clear();
    }
  }

  public InstantCommand incrementAnimationCommand() {
    return new InstantCommand(() -> incrementAnimation());
  }

  public InstantCommand setColorCommand(LedColors ledColor, LedSections ledSection) {
    return new InstantCommand(() -> setSolidColor(ledColor, ledSection));
  }

  public InstantCommand setSolidColorCommand(LedColors ledColor) {
    return new InstantCommand(() -> setSolidColor(ledColor, LedSections.ALL));
  }

  public InstantCommand changeAnimationCommand(LedModes ledMode) {
    return new InstantCommand(() -> changeAnimation(ledMode));
  }
}
