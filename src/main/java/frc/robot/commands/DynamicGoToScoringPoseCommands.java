package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.constants.enums.ScoringLocations;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LedController;

public class DynamicGoToScoringPoseCommands {
  public static CommandBase manual(
      Drivetrain drivetrain, Controlboard controlboard, LedController ledController
  ) {
    return Commands
        .either(
            new DynamicGoToScoringLocationCommand(
                drivetrain,
                controlboard,
                false,
                new Transform2d(new Translation2d(-0.1, 0.0), Rotation2d.fromDegrees(0)),
                ScoringLocations.UNKNOWN_SCORING_LOCATION
            ),
            Commands.either(
                new DynamicGoToScoringLocationCommand(
                    drivetrain,
                    controlboard,
                    false,
                    new Transform2d(new Translation2d(-0.25, 0.0), Rotation2d.fromDegrees(0)),
                    ScoringLocations.UNKNOWN_SCORING_LOCATION
                ),
                new DynamicGoToScoringLocationCommand(
                    drivetrain,
                    controlboard,
                    false,
                    new Transform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)),
                    ScoringLocations.UNKNOWN_SCORING_LOCATION
                ),
                () -> {
                  return controlboard.getDesiredScoringLevel() == ScoringLevels.SCORING_LEVEL_1;
                }
            ),
            () -> { return controlboard.getHeldGamePiece() == GamePieceType.CUBE_GAME_PIECE; }
        )
        .withTimeout(Constants.ALIGN_SHOT_TIMEOUT_S);
  }
}
