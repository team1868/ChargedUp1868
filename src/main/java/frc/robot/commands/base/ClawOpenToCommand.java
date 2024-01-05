package frc.robot.commands.base;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.subsystems.Claw;

public class ClawOpenToCommand extends CommandBase {
  private final Claw _claw;
  private ClawPositions _clawPosition;
  private final int _lowerBound, _upperBound;

  public ClawOpenToCommand(Claw claw, ClawPositions clawPosition, int upperBound, int lowerBound) {
    _claw = claw;
    _clawPosition = clawPosition;
    _upperBound = upperBound;
    _lowerBound = -Math.abs(lowerBound);

    addRequirements(claw);
  }

  public ClawOpenToCommand(Claw claw, ClawPositions clawPosition, int tolerance) {
    this(claw, clawPosition, tolerance, tolerance);
  }

  public ClawOpenToCommand(Claw claw, ClawPositions clawPosition) {
    this(
        claw, clawPosition, ClawPositions.CLAW_TOLERANCE_TICKS, ClawPositions.CLAW_TOLERANCE_TICKS
    );
  }

  @Override
  public void initialize() {
    _claw.setTargetPosition(_clawPosition);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (!Constants.isPneumaticClaw) {
      int pos = _claw.getClawPosition();
      if (interrupted || pos < ClawPositions.CLAW_MIN_TICKS.ticks
          || pos > ClawPositions.CLAW_OPEN_POS_TICKS.ticks) {
        _claw.setClawPower(0.0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return _claw.inTolerance(_lowerBound, _upperBound);
  }
}
