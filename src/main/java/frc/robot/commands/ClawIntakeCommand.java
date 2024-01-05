package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.base.ClawOpenToCommand;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.ClawConfs.IntakeSpeed;
import frc.robot.constants.enums.ClawDirections;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.LedController;

public class ClawIntakeCommand {
  public static CommandBase ClawIntake(
      Claw claw, Controlboard controlboard, LedController ledController
  ) {
    // REQUIRES DESIRED GAME PIECE TO BE CORRECT
    return Commands
        .sequence(
            // Fold claw out
            new ClawOpenToCommand(claw, ClawPositions.CLAW_OPEN_POS_TICKS),
            // Different intake speeds per game piece type
            ClawRollersCommands.desiredPieceCommand(
                claw,
                controlboard,
                IntakeSpeed.CUBE_POWER,
                ClawDirections.CLAW_ROLLERS_CUBE_IN,
                IntakeSpeed.CONE_POWER,
                ClawDirections.CLAW_ROLLERS_CONE_IN
            ),
            // wait for rollers to start
            ClawRollersCommands.waitSequenceCommand(claw, controlboard),
            LedCommands.dynamicBlinkLedCommand(ledController, controlboard),
            ClawRollersCommands.desiredPieceStallCommand(claw, controlboard)
        )
        .finallyDo((boolean interrupted) -> {
          claw.setClawRollersStall(controlboard.getDesiredGamePiece());
        });
  }
}
