package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Ports;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.HPIntakeStations;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.constants.enums.ScoringLocations;
import frc.robot.utils.ControlboardViz;

public class Controlboard {
  public CommandXboxController _xboxDrive = new CommandXboxController(Ports.DRIVER_XBOX_USB_PORT);
  // TODO: change xbox op to PS3 or equivalent non-xbox controller
  public CommandPS4Controller _ps4Op = new CommandPS4Controller(Ports.OPERATOR_PS4_USB_PORT);
  // frc2::CommandXboxController _xboxPit{PIT_XBOX_USB_PORT};

  private Alliance _alliance;
  private ScoringLocations _desiredScoringLocation = Constants.DEFAULT_SCORING_LOCATION;
  private ScoringLevels _desiredScoringLevel = Constants.DEFAULT_SCORING_LEVEL;
  private frc.robot.constants.enums.HPIntakeStations _desiredIntakeLocation =
      Constants.DEFAULT_INTAKE_STATION;
  private GamePieceType _desiredGamePiece = Constants.DEFAULT_GAME_PIECE;
  private GamePieceType _heldGamePiece = GamePieceType.UNKNOWN_GAME_PIECE;
  private StowPositionStrategy _stowStrategy = StowPositionStrategy.PARTIAL_STOW_STRATEGY;
  private int _lastOperatorPOVPressed = -1;

  private Field2d _field;

  private boolean VIZCONF_TEXT = true;
  private boolean VIZCONF_GLASS = true;
  private boolean VIZCONF_FIELD = true;

  // display on text & glass, not on field (which isn't built yet anyway)
  ControlboardViz _viz = new ControlboardViz(_field, VIZCONF_TEXT, VIZCONF_GLASS, VIZCONF_FIELD);

  public Controlboard(Field2d field) {}

  public void configShuffleboard() {
    updateVisualizer();
    // frc::ShuffleboardTab& controlboardTab = frc::Shuffleboard::GetTab("Competition HUD");
    // _clawPowerEntry = controlboardTab.Add("Cone Shooter Power", "UNKNOWN HELD PIECE").GetEntry();
  }

  public void updateShuffleboard() {}

  public void updateVisualizer() {
    _viz.update(
        _desiredScoringLocation,
        _desiredScoringLevel,
        _desiredIntakeLocation,
        _desiredGamePiece,
        _heldGamePiece
    );
  }

  // Left is positive X in terms of field, negative so our controller aligns
  public double getDriveX() {
    return _alliance == Alliance.Red ? _xboxDrive.getLeftY() : -_xboxDrive.getLeftY();
  }

  public double getDriveY() {
    return _alliance == Alliance.Red ? _xboxDrive.getLeftX() : -_xboxDrive.getLeftX();
  }

  public double getRotX() {
    return -_xboxDrive.getRightX();
  }

  public double getRotY() {
    return _xboxDrive.getRightY();
  }

  public boolean getBackButtonPressed() {
    return _xboxDrive.getHID().getBackButtonPressed();
  }

  public void updateAlliance(Alliance alliance) {
    _alliance = alliance;
    _viz.updateAlliance(alliance);
  }

  public boolean isRedAlliance() {
    return _alliance == Alliance.Red;
  }

  public boolean isPieceInIntake(boolean beamBreak, boolean currentSpike) {
    if ((getDesiredGamePiece().isCube() && beamBreak)
        || (getDesiredGamePiece() == GamePieceType.CONE_GAME_PIECE && currentSpike)) {
      setHeldGamePiece();
      return true;
    } else {
      return false;
    }
  }

  public ScoringLocations getDesiredScoringLocation() {
    return _desiredScoringLocation;
  }

  public ScoringLevels getDesiredScoringLevel() {
    return _desiredScoringLevel;
  }

  public HPIntakeStations getDesiredIntakeLocation() {
    return _desiredIntakeLocation;
  }

  public GamePieceType getDesiredGamePiece() {
    return _desiredGamePiece;
  }

  public void setDesiredGamePiece(GamePieceType piece) {
    _desiredGamePiece = piece;
    updateVisualizer();
  }

  public GamePieceType getHeldGamePiece() {
    return _heldGamePiece;
  }

  public void setHeldGamePiece() {
    setHeldGamePiece(_desiredGamePiece);
  }

  public void setHeldGamePiece(GamePieceType piece) {
    _heldGamePiece = piece;
    updateVisualizer();
  }

  public void unsetHeldGamePiece() {
    setHeldGamePiece(GamePieceType.UNKNOWN_GAME_PIECE);
  }

  public StowPositionStrategy getStowStrategy() {
    return _stowStrategy;
  }

  public void setStowStrategy(StowPositionStrategy strategy) {
    _stowStrategy = strategy;
  }

  public void toggleStowStrategy() {
    _stowStrategy = (_stowStrategy == StowPositionStrategy.FULL_STOW_STRATEGY)
        ? StowPositionStrategy.PARTIAL_STOW_STRATEGY
        : StowPositionStrategy.FULL_STOW_STRATEGY;
  }

  public void driverRumble() {
    _xboxDrive.getHID().setRumble(RumbleType.kBothRumble, 1.0);
  }

  public void driverResetRumble() {
    _xboxDrive.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }

  public void incrementLevel() {
    _desiredScoringLevel = _desiredScoringLevel.getIncr();
    updateVisualizer();
  }

  public void decrementLevel() {
    _desiredScoringLevel = _desiredScoringLevel.getDecr();
    updateVisualizer();
  }

  public void incrementCol() {
    _desiredScoringLocation = _desiredScoringLocation.getIncr();
    updateVisualizer();
  }

  public void decrementCol() {
    _desiredScoringLocation = _desiredScoringLocation.getDecr();
    updateVisualizer();
  }

  public void swapDesiredGamePiece() {
    _desiredGamePiece =
        !_desiredGamePiece.isCube() ? GamePieceType.CUBE_GAME_PIECE : GamePieceType.CONE_GAME_PIECE;
    updateVisualizer();
  }

  public void swapIntakeLocation() {
    switch (_desiredIntakeLocation) {
      case DOUBLE_STATION_INNER:
        _desiredIntakeLocation = HPIntakeStations.DOUBLE_STATION_OUTER;
        break;
      case DOUBLE_STATION_OUTER:
      case SINGLE_STATION:
      case UNKNOWN_INTAKE_STATION:
      default:
        _desiredIntakeLocation = HPIntakeStations.DOUBLE_STATION_INNER;
        break;
    }
    _viz.update(
        _desiredScoringLocation,
        _desiredScoringLevel,
        _desiredIntakeLocation,
        _desiredGamePiece,
        _heldGamePiece
    );
  }

  public void updateOperatorSelections() {
    int prev = _lastOperatorPOVPressed;
    _lastOperatorPOVPressed = getOperatorPOV();
    if (prev == -1) {
      switch (_lastOperatorPOVPressed) {
        case 0:
          incrementLevel();
          break;
        case 180:
          decrementLevel();
          break;
        case 90:
          if (_alliance == Alliance.Red) {
            incrementCol();
          } else {
            decrementCol();
          }
          break;
        case 270:
          if (_alliance == Alliance.Red) {
            decrementCol();
          } else {
            incrementCol();
          }
          break;
        default:
          break;
      }
    }
  }

  public int getOperatorPOV() {
    return _ps4Op.getHID().getPOV();
  }

  public enum StowPositionStrategy { FULL_STOW_STRATEGY, PARTIAL_STOW_STRATEGY }

  public InstantCommand driverRumbleCommand() {
    return new InstantCommand(() -> driverRumble());
  }

  public InstantCommand driverResetRumbleCommand() {
    return new InstantCommand(() -> driverResetRumble());
  }

  public CommandBase swapDesiredGamePieceCommand() {
    return new InstantCommand(() -> swapDesiredGamePiece()).ignoringDisable(true);
  }

  public CommandBase swapIntakeLocationCommand() {
    return new InstantCommand(() -> swapIntakeLocation()).ignoringDisable(true);
  }

  public InstantCommand setHeldGamePieceCommand() {
    return new InstantCommand(() -> setHeldGamePiece());
  }

  public InstantCommand unsetHeldGamePieceCommand() {
    return new InstantCommand(() -> unsetHeldGamePiece());
  }
}
