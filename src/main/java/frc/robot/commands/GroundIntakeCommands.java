package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CommandUtils.ElevatorUtils;
import frc.robot.commands.base.ClawOpenToCommand;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.ClawConfs.IntakeSpeed;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.LedColors;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;

public class GroundIntakeCommands {
  public static CommandBase teleop(
      Claw claw, Elevator elevator, LedController ledController, Controlboard controlboard
  ) {
    return Commands
        .sequence(
            new ClawOpenToCommand(claw, ClawPositions.CLAW_OPEN_POS_TICKS),
            Commands.parallel(
                Commands.either(
                    ledController.setSolidColorCommand(LedColors.PURPLE),
                    ledController.setSolidColorCommand(LedColors.YELLOW),
                    () -> controlboard.getDesiredGamePiece() == GamePieceType.CUBE_GAME_PIECE
                ),
                ElevatorUtils.pieceSpecificElevatorRaiseToCommand(
                    claw,
                    controlboard,
                    elevator,
                    ElevatorPositions.ELEVATOR_GROUND_CUBE_INTAKE,
                    ElevatorPositions.ELEVATOR_GROUND_CONE_INTAKE
                )
            ),
            ClawIntakeCommand.ClawIntake(claw, controlboard, ledController)
        )
        .finallyDo((boolean interrupted) -> {
          claw.closeClawSolenoid();
          claw.setClawRollersStall(controlboard.getDesiredGamePiece());
          ledController.turnOffLeds(LedSections.ALL);
        });
  }

  public static CommandBase auton(
      Claw claw,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard,
      double timeout,
      GamePieceType piece
  ) {
    // if autotesting return none
    return Commands.parallel(
        ledController.setSolidColorCommand(LedColors.PURPLE),
        Commands
            .sequence(
                ClawRollersCommands.autonIn(
                    claw, piece, IntakeSpeed.CUBE_POWER, IntakeSpeed.CONE_POWER
                ),
                Commands.waitSeconds(ClawConfs.CLAW_ROLLER_SPINUP_TIME_S),
                Commands.waitUntil(() -> {
                  return controlboard.isPieceInIntake(
                      claw.getBeamBreak(),
                      claw.coneDetected(IntakeSpeed.CONE_DETECTION_CURRENT_THRESHOLD_A)
                  );
                }),
                ClawRollersCommands.autonIn(
                    claw, piece, IntakeSpeed.CUBE_POWER, IntakeSpeed.CONE_POWER
                )
            )
            .withTimeout(timeout)
            .beforeStarting(() -> { controlboard.setDesiredGamePiece(piece); })
            .finallyDo((boolean interrupted) -> { controlboard.setHeldGamePiece(); })
    );
  }

  public static CommandBase autonPrep(
      Claw claw,
      Elevator elevator,
      Controlboard controlboard,
      LedController ledController,
      GamePieceType piece
  ) {
    if (RobotAltModes.isAutoTuning)
      return Commands.none();

    return Commands.parallel(
        ledController.setSolidColorCommand(LedColors.RED_ORANGE),
        Commands.sequence(
            ClawRollersCommands.autonIn(
                claw, piece, IntakeSpeed.CUBE_POWER, IntakeSpeed.CONE_POWER
            ),
            new InstantCommand(() -> claw.openClawSolenoid())
        ),
        new ElevatorRaiseToCommand(
            elevator,
            controlboard,
            piece.isCube() ? ElevatorPositions.ELEVATOR_GROUND_CUBE_INTAKE
                           : ElevatorPositions.ELEVATOR_GROUND_CONE_INTAKE
        )
            .beforeStarting(() -> controlboard.setDesiredGamePiece(piece))
    );
  }

  public static CommandBase autonPost(
      Claw claw,
      Elevator elevator,
      Controlboard controlboard,
      LedController ledController,
      GamePieceType piece
  ) {
    return Commands.parallel(
        ledController.setSolidColorCommand(LedColors.YELLOW),
        ClawRollersCommands.autonIn(
            claw, piece, ClawConfs.CLAW_CUBE_INTAKE_RESETTLE, ClawConfs.CLAW_CONE_INTAKE_RESETTLE
        ),
        new InstantCommand(() -> claw.closeClawSolenoid()),
        StowCommands.auto(claw, controlboard, elevator)
    );
  }
}
