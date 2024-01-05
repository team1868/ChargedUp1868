package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.ClawDirections;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.subsystems.*;
import java.util.function.BooleanSupplier;

public class ClawRollersCommands {
  public static CommandBase on(Claw claw, ClawDirections clawDirection, double speed) {
    if (Constants.isKlaw) {
      switch (clawDirection) {
        case CLAW_ROLLERS_CONE_IN:
        case CLAW_ROLLERS_CUBE_OUT:
          return claw.setClawRollersPowerCommand(speed);
        case CLAW_ROLLERS_CONE_OUT:
        case CLAW_ROLLERS_CUBE_IN:
          return claw.setClawRollersPowerCommand(-speed);
        default:
          return claw.setClawRollersPowerCommand(0.0);
      }
    }
    return Commands.print("No Claw Rollers on this robot");
  }

  public static CommandBase conditionalCommand(
      Claw claw,
      Controlboard controlboard,
      double cubeSpeed,
      ClawDirections cubeDirection,
      double coneSpeed,
      ClawDirections coneDirection,
      BooleanSupplier condition
  ) {
    return Commands.either(
        ClawRollersCommands.on(claw, cubeDirection, cubeSpeed),
        ClawRollersCommands.on(claw, coneDirection, coneSpeed),
        condition
    );
  }

  public static CommandBase desiredPieceCommand(
      Claw claw,
      Controlboard controlboard,
      double cubeSpeed,
      ClawDirections cubeDirection,
      double coneSpeed,
      ClawDirections coneDirection
  ) {
    return ClawRollersCommands.conditionalCommand(
        claw,
        controlboard,
        cubeSpeed,
        cubeDirection,
        coneSpeed,
        coneDirection,
        () -> controlboard.getDesiredGamePiece().isCube()
    );
  }

  public static CommandBase desiredPieceStallCommand(Claw claw, Controlboard controlboard) {
    return desiredPieceCommand(
        claw,
        controlboard,
        ClawConfs.CLAW_CUBE_STALL_POWER,
        ClawDirections.CLAW_ROLLERS_CUBE_IN,
        ClawConfs.CLAW_CONE_STALL_POWER,
        ClawDirections.CLAW_ROLLERS_CONE_IN
    );
  }

  public static CommandBase heldPieceCommand(
      Claw claw,
      Controlboard controlboard,
      double cubeSpeed,
      ClawDirections cubeDirection,
      double coneSpeed,
      ClawDirections coneDirection
  ) {
    return conditionalCommand(
        claw,
        controlboard,
        cubeSpeed,
        cubeDirection,
        coneSpeed,
        coneDirection,
        () -> controlboard.getHeldGamePiece().isCube()
    );
  }

  public static CommandBase waitSequenceCommand(Claw claw, Controlboard controlboard) {
    return Commands.sequence(
        Commands.waitSeconds(ClawConfs.CLAW_ROLLER_SPINUP_TIME_S),
        Commands.waitUntil(
            () -> controlboard.isPieceInIntake(claw.getBeamBreak(), claw.coneDetected())
        ),
        Commands.either(
            Commands.waitSeconds(ClawConfs.CLAW_CUBE_SETTLE_TIME_S),
            Commands.waitSeconds(ClawConfs.CLAW_CONE_SETTLE_TIME_S),
            () -> controlboard.getDesiredGamePiece().isCube()
        )
    );
  }

  public static CommandBase autonIn(
      Claw claw, GamePieceType piece, double cubePower, double conePower
  ) {
    // If an unknown piece is here, we're calling this in a dynamic command (be worried)
    assert (piece != GamePieceType.UNKNOWN_GAME_PIECE);
    return piece.isCube()
        ? ClawRollersCommands.on(claw, ClawDirections.CLAW_ROLLERS_CUBE_IN, cubePower)
        : ClawRollersCommands.on(claw, ClawDirections.CLAW_ROLLERS_CONE_IN, conePower);
  }
}
