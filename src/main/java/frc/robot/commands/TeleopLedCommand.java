package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.HPIntakeStations;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.constants.enums.ScoringLocations;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.LedController;

public class TeleopLedCommand extends CommandBase {
  private final LedController _ledController;
  private final Controlboard _controlboard;
  private boolean _isIntaking;
  private boolean _wasIntaking;
  private GamePieceType _desiredGamePiece = GamePieceType.UNKNOWN_GAME_PIECE;
  private HPIntakeStations _desiredHPIntakeStations = HPIntakeStations.UNKNOWN_INTAKE_STATION;
  private ScoringLocations _desiredScoringLocation = ScoringLocations.UNKNOWN_SCORING_LOCATION;
  private ScoringLevels _desiredScoringLevel = ScoringLevels.UNKNOWN_SCORING_LEVEL;

  public TeleopLedCommand(LedController ledController, Controlboard controlboard) {
    _controlboard = controlboard;
    _ledController = ledController;

    addRequirements(ledController);
  }

  @Override
  public void execute() {
    GamePieceType temp = _controlboard.getHeldGamePiece();
    if (temp == GamePieceType.CONE_GAME_PIECE || temp.isCube()) {
      _isIntaking = false;
    } else if (temp.isUnknown()) {
      _isIntaking = true;
    } else {
      _isIntaking = !_isIntaking;
    }

    if (!(_isIntaking == _wasIntaking)
        || _desiredGamePiece != _controlboard.getDesiredGamePiece()) {
      _desiredGamePiece = _controlboard.getDesiredGamePiece();
      _ledController.setSolidColor(_desiredGamePiece.intakeColor, LedSections.ALL);
    }
    _wasIntaking = _isIntaking;
  }
}
