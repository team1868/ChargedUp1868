package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.constants.enums.ClawScoreLevels;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Controlboard;

public class ClawOuttakeCommand extends CommandBase {
  private final Claw _claw;
  private final Controlboard _controlboard;
  private ClawScoreLevels _clawScoreLevel;
  private int _counter;
  private boolean _isAuto;

  public ClawOuttakeCommand(
      Claw claw, Controlboard controlboard, boolean isAuto, ClawScoreLevels clawScoreLevel
  ) {
    _claw = claw;
    _controlboard = controlboard;
    _clawScoreLevel = clawScoreLevel;
    _isAuto = isAuto;

    addRequirements(claw);
  }

  public ClawOuttakeCommand(Claw claw, Controlboard controlboard) {
    this(claw, controlboard, false, ClawScoreLevels.CLAW_SCORE_UNKNOWN);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    _claw.setClawRollersPower(
        _clawScoreLevel == ClawScoreLevels.CLAW_SCORE_UNKNOWN
            ? _controlboard.getDesiredScoringLevel().getPower(_controlboard.getHeldGamePiece())
            : _clawScoreLevel.power
    );
  }

  @Override
  public void end(boolean interrupted) {
    _controlboard.unsetHeldGamePiece();
    _claw.setClawRollersPower(0.0);
    if (_isAuto)
      _claw.closeClawSolenoid();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
