package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.enums.ClawDirections;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.subsystems.*;

public class PrepToScoreCommands {
  public static CommandBase teleop(
      Claw claw, Controlboard controlboard, Elevator elevator, LedController ledController
  ) {
    return teleop(
        claw, controlboard, elevator, ledController, ElevatorPositions.ELEVATOR_UNKNOWN_POSITION
    );
  }

  public static CommandBase teleop(
      Claw claw,
      Controlboard controlboard,
      Elevator elevator,
      LedController ledController,
      ElevatorPositions position
  ) {
    return teleop(
        claw, controlboard, elevator, ledController, position, GamePieceType.UNKNOWN_GAME_PIECE
    );
  }

  // In paralell reseat game piece, raise elevator --> deploy claw, then wait claw extension time
  public static CommandBase teleop(
      Claw claw,
      Controlboard controlboard,
      Elevator elevator,
      LedController ledController,
      ElevatorPositions position,
      GamePieceType piece
  ) {
    return Commands.parallel(
        // Deploy claw once elevator is at full height
        new ElevatorRaiseToCommand(elevator, controlboard, position)
            .finallyDo((boolean interrupted) -> {
              if (controlboard.getDesiredScoringLevel() != ScoringLevels.SCORING_LEVEL_0)
                claw.openClawSolenoid();
            }),
        ClawRollersCommands.conditionalCommand(
            claw,
            controlboard,
            ClawConfs.CLAW_CUBE_INTAKE_RESETTLE,
            ClawDirections.CLAW_ROLLERS_CUBE_IN,
            ClawConfs.CLAW_CONE_INTAKE_RESETTLE,
            ClawDirections.CLAW_ROLLERS_CONE_IN,
            (() -> {
              return piece != GamePieceType.UNKNOWN_GAME_PIECE
                  ? piece.isCube()
                  : controlboard.getHeldGamePiece().isCube();
            })
        )
    );
  }

  public static CommandBase autonomous(
      Claw claw,
      Controlboard controlboard,
      Elevator elevator,
      ElevatorPositions position,
      GamePieceType piece
  ) {
    return Commands.sequence(
        Commands.deadline(
            new ElevatorRaiseToCommand(elevator, controlboard, position),
            ClawRollersCommands.autonIn(
                claw,
                piece,
                ClawConfs.CLAW_CUBE_INTAKE_RESETTLE,
                ClawConfs.CLAW_CONE_INTAKE_RESETTLE
            )
        ),
        // because it's autonomous we can do a simple ternary
        piece.isCube()
            ? Commands.none()
            : claw.openClawCommand().andThen(Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_OUT_TIME_S))
    );
  }
}
