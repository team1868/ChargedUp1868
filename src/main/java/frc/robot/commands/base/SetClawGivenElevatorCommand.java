package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Elevator;

public class SetClawGivenElevatorCommand extends CommandBase {
  private final Claw _claw;
  private final Controlboard _controlboard;
  private final Elevator _elevator;
  private ClawPositions _clawPosition;
  private int _elevatorPosition;
  private boolean _higher;

  public SetClawGivenElevatorCommand(
      Claw claw,
      Controlboard controlboard,
      Elevator elevator,
      ClawPositions desiredClawPosition,
      int elevatorPosition,
      boolean higher
  ) {
    _claw = claw;
    _controlboard = controlboard;
    _elevator = elevator;
    _clawPosition = desiredClawPosition;
    _elevatorPosition = elevatorPosition;
    _higher = higher;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      if (_clawPosition == ClawPositions.CLAW_MAX_TICKS) {
        _claw.openClawSolenoid();
      } else {
        _claw.closeClawSolenoid();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return _higher ? _elevator.getCurrentHeight() >= _elevatorPosition
                   : _elevator.getCurrentHeight() <= _elevatorPosition;
  }
}
