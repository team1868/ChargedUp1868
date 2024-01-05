package frc.robot.commands.base;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConfs;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Elevator;

public class ElevatorRaiseToCommand extends CommandBase {
  private final Elevator _elevator;
  private final Controlboard _controlboard;
  private final int _lowerTolerance, _upperTolerance;
  private ElevatorPositions _elevatorPosition;
  private ElevatorPositions _configuredPosition;
  private boolean _startedMovement;
  private static GenericPublisher _finishedTimeEntry;
  private Timer _timer = new Timer();

  public ElevatorRaiseToCommand(
      Elevator elevator,
      Controlboard controlboard,
      ElevatorPositions elevatorPosition,
      int lowerTolerance,
      int upperTolerance
  ) {
    _elevator = elevator;
    _controlboard = controlboard;
    _elevatorPosition = elevatorPosition;
    _lowerTolerance = -Math.abs(lowerTolerance);
    _upperTolerance = upperTolerance;

    addRequirements(elevator);
    if (_finishedTimeEntry == null && RobotAltModes.isElevatorTuningMode) {
      ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");
      _finishedTimeEntry =
          tuningTab.add("Time finished elevator raise", 0.0).withPosition(1, 1).getEntry();
    }
  }

  public ElevatorRaiseToCommand(
      Elevator elevator, Controlboard controlboard, ElevatorPositions elevatorPosition
  ) {
    this(
        elevator,
        controlboard,
        elevatorPosition,
        ElevatorConfs.ELEVATOR_TOLERANCE_TICKS,
        ElevatorConfs.ELEVATOR_TOLERANCE_TICKS
    );
  }

  public ElevatorRaiseToCommand(Elevator elevator, Controlboard controlboard) {
    this(
        elevator,
        controlboard,
        ElevatorPositions.ELEVATOR_UNKNOWN_POSITION,
        ElevatorConfs.ELEVATOR_TOLERANCE_TICKS,
        ElevatorConfs.ELEVATOR_TOLERANCE_TICKS
    );
  }

  @Override
  public void initialize() {
    _startedMovement = false;
    _configuredPosition = (_elevatorPosition.isUnknown())
        ? _elevator.getElevatorScoringHeight(
            _controlboard.getHeldGamePiece(), _controlboard.getDesiredScoringLevel()
        )
        : _elevatorPosition;
  }

  @Override
  public void execute() {
    // If elevator has not started moving, check for legal states
    if (!_startedMovement) {
      _startedMovement = true;
      _elevator.setElevatorProfile(_configuredPosition);
      if (RobotAltModes.isElevatorTuningMode) {
        _timer.start();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    int height = _elevator.getCurrentHeight();

    // Emergency shutdown
    if (height < ElevatorPositions.ELEVATOR_MIN_TICKS.ticks
        || height > ElevatorPositions.ELEVATOR_MAX_TICKS.ticks) {
      _elevator.setPower(0.0);
    }
    if (RobotAltModes.isElevatorTuningMode) {
      _finishedTimeEntry.setDouble(_timer.get());
    }
  }

  @Override
  public boolean isFinished() {
    // if going up shutdown if we go over max ticks
    // if going down shutdown if we go under min ticks
    return _elevator.inTolerance(_lowerTolerance, _upperTolerance);
  }
}
