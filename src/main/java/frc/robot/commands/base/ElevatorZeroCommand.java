package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConfs;
import frc.robot.subsystems.Elevator;

public class ElevatorZeroCommand extends CommandBase {
  private final Elevator _elevator;
  private int _prev;
  private int _count;

  public ElevatorZeroCommand(Elevator elevator) {
    _elevator = elevator;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    _elevator.setPower(ElevatorConfs.ELEVATOR_ZEROING_SPEED_MPS);
    _prev = _elevator.getCurrentHeight();
    _count = 0;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    _elevator.setPower(0.0);
    if (!interrupted)
      _elevator.setEncoder(0);
  }

  @Override
  public boolean isFinished() {
    int currHeight = _elevator.getCurrentHeight();

    if (_prev - currHeight > ElevatorConfs.ELEVATOR_ZEROING_STALL_TOLERANCE) {
      _prev = currHeight;
      _count = 0;
      return false;
    } else if (_count++ < ElevatorConfs.ELEVATOR_ZEROING_STALL_LOOPS) {
      _prev = currHeight;
      return false;
    } else {
      return true;
    }
  }
}
