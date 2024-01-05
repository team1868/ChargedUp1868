package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Elevator;

public class CommandUtils {
  public static final class ElevatorUtils {
    public static CommandBase pieceSpecificElevatorRaiseToCommand(
        Claw claw,
        Controlboard controlboard,
        Elevator elevator,
        ElevatorPositions cubeElevatorPositions,
        ElevatorPositions coneElevatorPositions
    ) {
      return Commands.either(
          new ElevatorRaiseToCommand(elevator, controlboard, cubeElevatorPositions),
          new ElevatorRaiseToCommand(elevator, controlboard, coneElevatorPositions),
          () -> controlboard.getDesiredGamePiece().isCube()
      );
    }
  }
}
