package frc.robot.commands.score;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ClawRollersCommands;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.enums.ClawDirections;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.ScoringLevels.Consts;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Elevator;

public class WireCoverScoreCommand {
  CommandBase score(
      Claw claw,
      Controlboard controlboard,
      Elevator elevator,
      double clawPower,
      double elevatorThreshold
  ) {
    return Commands
        .deadline(
            Commands.sequence(
                Commands.waitUntil(() -> elevator.getCurrentHeight() > elevatorThreshold),
                claw.setClawPowerCommand(clawPower),
                Commands.waitSeconds(Consts.AUTO_YEET_TIMEOUT_S)
            ),
            new ElevatorRaiseToCommand(
                elevator, controlboard, ElevatorPositions.ELEVATOR_LEVEL_2_CONE
            )
        )
        .finallyDo(
            (boolean interrupted
            ) -> elevator.setElevatorProfile(ElevatorPositions.ELEVATOR_STOW_TICKS)
        );
  }

  CommandBase prepScore(
      Claw claw, Controlboard controlboard, Elevator elevator, GamePieceType piece
  ) {
    return Commands.parallel(
        ClawRollersCommands.conditionalCommand(
            claw,
            controlboard,
            ClawConfs.CLAW_CUBE_INTAKE_RESETTLE,
            ClawDirections.CLAW_ROLLERS_CUBE_IN,
            ClawConfs.CLAW_CONE_INTAKE_RESETTLE,
            ClawDirections.CLAW_ROLLERS_CONE_IN,
            (() -> piece.isCube())
        ),
        new ElevatorRaiseToCommand(elevator, controlboard, ElevatorPositions.ELEVATOR_STOW_TICKS),
        claw.closeClawCommand()
    );
  }

  CommandBase postScore(Claw claw, Controlboard controlboard, Elevator elevator) {
    return Commands.parallel(
        new ElevatorRaiseToCommand(elevator, controlboard, ElevatorPositions.ELEVATOR_STOW_TICKS),
        new InstantCommand(() -> claw.setClawRollersPower(0.0))
    );
  }
}
