package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.ClawOuttakeCommand;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.ClawDirections;
import frc.robot.constants.enums.ClawScoreLevels;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;

public class AutonomousScoreCommands {
  public static CommandBase score(
      Elevator elevator,
      Controlboard controlboard,
      Claw claw,
      LedController ledController,
      Drivetrain drivetrain,
      ElevatorPositions position,
      ClawScoreLevels clawScoreLevel,
      GamePieceType piece,
      double timeout_SEC
  ) {
    if (RobotAltModes.isAutoTuning) {
      return Commands.none();
    } else {
      return Commands
          .sequence(
              ledController.setSolidColorCommand(LedColors.BLUE),
              PrepToScoreCommands.autonomous(claw, controlboard, elevator, position, piece),
              Commands.either(
                  claw.setClawRollersPowerCommand(1.0).andThen(
                      Commands.waitSeconds(ScoringLevels.Consts.AUTO_CUBE_SHOT_TIMEOUT_S)
                  ),
                  new ClawOuttakeCommand(claw, controlboard, true, clawScoreLevel)
                      .withTimeout(ScoringLevels.Consts.AUTO_CONE_SHOT_TIMEOUT_S)
                      .andThen(Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S)),
                  () -> piece.isCube()
              ),
              Commands.parallel(
                  claw.clawRollersOffCommand(),
                  piece.isCube() ? Commands.none()
                                 : Commands.waitSeconds(0.15).andThen(
                                     StowCommands.auto(claw, controlboard, elevator)
                                 )
              )

          )
          .withTimeout(timeout_SEC)
          .finallyDo((boolean interrupted) -> {
            controlboard.unsetHeldGamePiece();
            claw.setClawRollersPower(0.0);
          });
    }
  }

  public static CommandBase prep(
      Elevator elevator,
      Controlboard controlboard,
      Claw claw,
      LedController ledController,
      Drivetrain drivetrain,
      ElevatorPositions position,
      GamePieceType piece
  ) {
    // TODO update this to optimize auto.
    if (RobotAltModes.isAutoTuning) {
      return Commands.none();
    }
    return Commands
        .sequence(
            ClawRollersCommands.autonIn(
                claw, piece, ClawConfs.CLAW_CUBE_AUTO_STALL_POWER, ClawConfs.CLAW_CONE_STALL_POWER
            ),
            Commands.parallel(
                // todo do according to held game piece
                ledController.setSolidColorCommand(LedColors.GREEN),
                PrepToScoreCommands.autonomous(claw, controlboard, elevator, position, piece)
            )
        )
        .beforeStarting(() -> drivetrain.setAutoPrepScore(true));
  }

  public static CommandBase post(
      Elevator elevator, Controlboard controlboard, Claw claw, LedController ledController
  ) {
    if (RobotAltModes.isAutoTuning) {
      return Commands.none();
    } else {
      // TODO update this to optimize auto
      // All subsystems passed as parameters, just so anything that is running is canceled.
      return Commands.sequence(
          Commands.waitSeconds(0.15),
          StowCommands.auto(claw, controlboard, elevator),
          ledController.setSolidColorCommand(LedColors.PURPLE)
      );
    }
  }

  public static CommandBase yeet(
      Elevator elevator, Controlboard controlboard, Claw claw, ClawScoreLevels clawPower
  ) {
    return Commands.sequence(
        new ElevatorRaiseToCommand(elevator, controlboard, ElevatorPositions.ELEVATOR_LEVEL_1_CUBE)
            .asProxy(),
        claw.setClawRollersPowerCommand(clawPower.power).asProxy(),
        Commands.waitSeconds(0.2).asProxy(),
        new ElevatorRaiseToCommand(elevator, controlboard, ElevatorPositions.ELEVATOR_STOW_TICKS)
            .asProxy()
    );
  }
}
