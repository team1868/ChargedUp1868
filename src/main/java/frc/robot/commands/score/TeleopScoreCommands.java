package frc.robot.commands.score;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DynamicGoToScoringLocationCommand;
import frc.robot.commands.LedCommands;
import frc.robot.commands.PrepToScoreCommands;
import frc.robot.commands.StowCommands;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.commands.base.ClawOuttakeCommand;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.Constants;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.constants.enums.ScoringLocations;
import frc.robot.constants.enums.Zones;
import frc.robot.subsystems.*;
import java.util.HashMap;
import java.util.Map;

public class TeleopScoreCommands {
  public static Map<Object, Command> scoringCommands;
  public static Map<Object, Command> automaticScoringCommands;

  public static CommandBase manual(
      ScoringLevels level,
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard
  ) {
    if (scoringCommands == null) {
      scoringCommands = new HashMap<Object, Command>();
      scoringCommands.put(0, level0(claw, drivetrain, elevator, ledController, controlboard));
      scoringCommands.put(1, level1(claw, drivetrain, elevator, ledController, controlboard));
      scoringCommands.put(2, level2(claw, drivetrain, elevator, ledController, controlboard));
    }
    return Commands.select(
        scoringCommands,
        ()
            -> level == ScoringLevels.UNKNOWN_SCORING_LEVEL
            ? Math.min(controlboard.getDesiredScoringLevel().id, 2)
            : level.id
    );
  }

  public static CommandBase level0(
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard
  ) {
    return Commands.sequence(
        Commands.deadline(
            Commands.sequence(
                prep(ledController, controlboard),
                Commands.either(
                    PrepToScoreCommands.teleop(
                        claw,
                        controlboard,
                        elevator,
                        ledController,
                        ElevatorPositions.ELEVATOR_LEVEL_0_CUBE,
                        GamePieceType.CUBE_GAME_PIECE
                    ),
                    PrepToScoreCommands.teleop(
                        claw,
                        controlboard,
                        elevator,
                        ledController,
                        ElevatorPositions.ELEVATOR_LEVEL_0_CONE,
                        GamePieceType.CONE_GAME_PIECE
                    ),
                    () -> controlboard.getHeldGamePiece().isCube()
                ),
                confirmAndEject(claw, ledController, controlboard)
            ),
            driveAndSnap(drivetrain, elevator, controlboard)
        ),
        Commands
            .deadline(
                StowCommands.tele(claw, controlboard, elevator),
                new TeleopSwerveCommand(
                    drivetrain, elevator, controlboard, DriveModes.FIELD_RELATIVE
                )
            )
            .finallyDo((boolean interrupted) -> {
              if (!interrupted) {
                controlboard.setHeldGamePiece(GamePieceType.UNKNOWN_GAME_PIECE);
              }
            })
    );
  }

  public static CommandBase level1(
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard
  ) {
    return Commands.sequence(
        Commands.deadline(
            Commands.sequence(
                prep(ledController, controlboard),
                Commands.either(
                    PrepToScoreCommands.teleop(
                        claw,
                        controlboard,
                        elevator,
                        ledController,
                        ElevatorPositions.ELEVATOR_LEVEL_1_CUBE,
                        GamePieceType.CUBE_GAME_PIECE
                    ),
                    Commands.sequence(
                        claw.openClawCommand(),
                        PrepToScoreCommands.teleop(
                            claw,
                            controlboard,
                            elevator,
                            ledController,
                            ElevatorPositions.ELEVATOR_LEVEL_1_CONE,
                            GamePieceType.CONE_GAME_PIECE
                        )
                    ),
                    () -> controlboard.getHeldGamePiece().isCube()
                ),
                confirmAndEject(claw, ledController, controlboard)
            ),
            driveAndSnap(drivetrain, elevator, controlboard)
        ),
        backoutAndStow(claw, drivetrain, elevator, controlboard)
    );
  }

  public static CommandBase level2(
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard
  ) {
    return Commands.sequence(
        Commands.deadline(
            Commands.sequence(
                prep(ledController, controlboard),
                Commands.either(
                    Commands.sequence(
                        PrepToScoreCommands.teleop(
                            claw,
                            controlboard,
                            elevator,
                            ledController,
                            ElevatorPositions.ELEVATOR_LEVEL_2_CUBE,
                            GamePieceType.CUBE_GAME_PIECE
                        ),
                        claw.openClawCommand(),
                        Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_OUT_TIME_S)
                    ),
                    PrepToScoreCommands.teleop(
                        claw,
                        controlboard,
                        elevator,
                        ledController,
                        ElevatorPositions.ELEVATOR_LEVEL_2_CONE,
                        GamePieceType.CONE_GAME_PIECE
                    ),
                    () -> controlboard.getHeldGamePiece().isCube()
                ),
                Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_OUT_TIME_S)
                    .beforeStarting(claw.openClawCommand()),
                confirmAndEject(claw, ledController, controlboard)
            ),
            driveAndSnap(drivetrain, elevator, controlboard)
        ),
        backoutAndStow(claw, drivetrain, elevator, controlboard)
    );
  }

  public static CommandBase automatic(
      ScoringLevels level,
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard
  ) {
    if (automaticScoringCommands == null) {
      automaticScoringCommands = new HashMap<Object, Command>();
      automaticScoringCommands.put(
          0, automaticLevel0(claw, drivetrain, elevator, ledController, controlboard)
      );
      automaticScoringCommands.put(
          1, automaticLevel1(claw, drivetrain, elevator, ledController, controlboard)
      );
      automaticScoringCommands.put(
          2, automaticLevel2(claw, drivetrain, elevator, ledController, controlboard)
      );
    }
    return Commands.select(
        automaticScoringCommands,
        ()
            -> level == ScoringLevels.UNKNOWN_SCORING_LEVEL
            ? Math.min(controlboard.getDesiredScoringLevel().id, 2)
            : level.id
    );
  }

  public static CommandBase automaticLevel0(
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard
  ) {
    return Commands
        .sequence(
            prep(ledController, controlboard, LedColors.BLUE),
            new DynamicGoToScoringLocationCommand(
                drivetrain,
                controlboard,
                false,
                Constants.SCORING_PREALIGN_RED_OFFSET,
                ScoringLocations.UNKNOWN_SCORING_LOCATION,
                Units.inchesToMeters(2),
                Units.inchesToMeters(5),
                Rotation2d.fromDegrees(10.0)
            ),
            PrepToScoreCommands.teleop(claw, controlboard, elevator, ledController),
            Commands.deadline(
                Commands.waitUntil(controlboard._xboxDrive.b()),
                new TeleopSwerveCommand(
                    drivetrain,
                    elevator,
                    controlboard,
                    DriveModes.SNAP_TO_ANGLE,
                    Constants.TRANSLATION_MAX_TRIM_SPEED_MPS,
                    Constants.ANGLE_MAX_TRIM_SPEED_DPS
                )
                    .beforeStarting(() -> {
                      controlboard.driverResetRumble();
                      drivetrain.setSnapScoringAngle();
                    })
                    .finallyDo((boolean interrupted) -> controlboard.driverRumble())
            ),
            LedCommands.dynamicBlinkLedCommand(ledController, controlboard),
            Commands.either(
                new ClawOuttakeCommand(claw, controlboard)
                    .withTimeout(ScoringLevels.Consts.TELEOP_CUBE_SHOT_TIMEOUT_S)
                    .andThen(Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S)),
                new ClawOuttakeCommand(claw, controlboard)
                    .withTimeout(ScoringLevels.Consts.TELEOP_YEET_TIMEOUT_S),
                () -> controlboard.getHeldGamePiece().isCube()
            ),

            Commands.deadline(
                Commands.sequence(
                    claw.closeClawCommand(),
                    Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S * 2),
                    StowCommands.tele(claw, controlboard, elevator)
                ),
                Commands.sequence(
                    drivetrain.forceAllianceBasedFieldRelativeMovementCommand(0.5, 0.0, 0.5),
                    new TeleopSwerveCommand(
                        drivetrain, elevator, controlboard, DriveModes.FIELD_RELATIVE
                    )
                )
            )

        )
        .beforeStarting(() -> {
          controlboard.driverRumble();
          ledController.setOverride(true);
        })
        .finallyDo((boolean interrupted) -> {
          controlboard.driverResetRumble();
          ledController.setOverride(false);
          ledController.turnOffLeds(LedSections.ALL);
        })
        .unless(() -> {
          return drivetrain.isRedAlliance()
              ? (drivetrain.getPose().getX() <= Zones.LocationMath.COMMUNITY_ZONE_RED_MIN_X_M)
              : (drivetrain.getPose().getX() >= Zones.LocationMath.COMMUNITY_ZONE_BLUE_MAX_X_M);
        });
  }

  public static CommandBase automaticLevel1(
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard
  ) {
    return Commands
        .sequence(
            prep(ledController, controlboard, LedColors.BLUE),
            new DynamicGoToScoringLocationCommand(
                drivetrain,
                controlboard,
                false,
                Constants.SCORING_PREALIGN_RED_OFFSET,
                ScoringLocations.UNKNOWN_SCORING_LOCATION,
                Units.inchesToMeters(2),
                Units.inchesToMeters(5),
                Rotation2d.fromDegrees(10.0)
            ),
            claw.openClawCommand().unless(
                () -> controlboard.getDesiredScoringLevel() != ScoringLevels.SCORING_LEVEL_1
            ),
            PrepToScoreCommands.teleop(claw, controlboard, elevator, ledController),
            Commands.deadline(
                Commands.waitUntil(controlboard._xboxDrive.b()),
                new TeleopSwerveCommand(
                    drivetrain,
                    elevator,
                    controlboard,
                    DriveModes.SNAP_TO_ANGLE,
                    Constants.TRANSLATION_MAX_TRIM_SPEED_MPS,
                    Constants.ANGLE_MAX_TRIM_SPEED_DPS
                )
                    .beforeStarting(() -> {
                      controlboard.driverResetRumble();
                      drivetrain.setSnapScoringAngle();
                    })
                    .finallyDo((boolean interrupted) -> controlboard.driverRumble())
            ),
            LedCommands.dynamicBlinkLedCommand(ledController, controlboard),
            Commands.either(
                new ClawOuttakeCommand(claw, controlboard)
                    .withTimeout(ScoringLevels.Consts.TELEOP_CUBE_SHOT_TIMEOUT_S)
                    .andThen(Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S)),
                new ClawOuttakeCommand(claw, controlboard)
                    .withTimeout(ScoringLevels.Consts.TELEOP_CONE_SHOT_TIMEOUT_S)
                    .andThen(Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S)),
                () -> controlboard.getHeldGamePiece().isCube()
            ),

            Commands.deadline(
                Commands.sequence(
                    claw.closeClawCommand(),
                    Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S * 2),
                    StowCommands.tele(claw, controlboard, elevator)
                ),
                Commands.sequence(
                    drivetrain.forceAllianceBasedFieldRelativeMovementCommand(0.5, 0.0, 0.5),
                    new TeleopSwerveCommand(
                        drivetrain, elevator, controlboard, DriveModes.FIELD_RELATIVE
                    )
                )
            )

        )
        .beforeStarting(() -> {
          controlboard.driverRumble();
          ledController.setOverride(true);
        })
        .finallyDo((boolean interrupted) -> {
          controlboard.driverResetRumble();
          ledController.setOverride(false);
          ledController.turnOffLeds(LedSections.ALL);
        })
        .unless(() -> {
          return drivetrain.isRedAlliance()
              ? (drivetrain.getPose().getX() <= Zones.LocationMath.COMMUNITY_ZONE_RED_MIN_X_M)
              : (drivetrain.getPose().getX() >= Zones.LocationMath.COMMUNITY_ZONE_BLUE_MAX_X_M);
        });
  }

  public static CommandBase automaticLevel2(
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard
  ) {
    return Commands
        .sequence(
            prep(ledController, controlboard, LedColors.BLUE),
            new DynamicGoToScoringLocationCommand(
                drivetrain,
                controlboard,
                false,
                Constants.SCORING_PREALIGN_RED_OFFSET,
                ScoringLocations.UNKNOWN_SCORING_LOCATION,
                Units.inchesToMeters(2),
                Units.inchesToMeters(5),
                Rotation2d.fromDegrees(10.0)
            ),
            PrepToScoreCommands.teleop(claw, controlboard, elevator, ledController),
            Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_OUT_TIME_S)
                .beforeStarting(claw.openClawCommand())
                .unless(
                    () -> controlboard.getDesiredScoringLevel() != ScoringLevels.SCORING_LEVEL_2
                ),
            Commands.deadline(
                Commands.waitUntil(controlboard._xboxDrive.b()),
                new TeleopSwerveCommand(
                    drivetrain,
                    elevator,
                    controlboard,
                    DriveModes.SNAP_TO_ANGLE,
                    Constants.TRANSLATION_MAX_TRIM_SPEED_MPS,
                    Constants.ANGLE_MAX_TRIM_SPEED_DPS
                )
                    .beforeStarting(() -> {
                      controlboard.driverResetRumble();
                      drivetrain.setSnapScoringAngle();
                    })
                    .finallyDo((boolean interrupted) -> controlboard.driverRumble())
            ),
            LedCommands.dynamicBlinkLedCommand(ledController, controlboard),
            Commands.either(
                new ClawOuttakeCommand(claw, controlboard)
                    .withTimeout(ScoringLevels.Consts.TELEOP_CUBE_SHOT_TIMEOUT_S)
                    .andThen(Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S)),
                new ClawOuttakeCommand(claw, controlboard)
                    .withTimeout(ScoringLevels.Consts.TELEOP_CONE_SHOT_TIMEOUT_S)
                    .andThen(Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S)),
                () -> controlboard.getHeldGamePiece().isCube()
            ),

            Commands.deadline(
                Commands.sequence(
                    claw.closeClawCommand(),
                    Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S * 2),
                    StowCommands.tele(claw, controlboard, elevator)
                ),
                Commands.sequence(
                    drivetrain.forceAllianceBasedFieldRelativeMovementCommand(0.5, 0.0, 0.5),
                    new TeleopSwerveCommand(
                        drivetrain, elevator, controlboard, DriveModes.FIELD_RELATIVE
                    )
                )
            )

        )
        .beforeStarting(() -> {
          controlboard.driverRumble();
          ledController.setOverride(true);
        })
        .finallyDo((boolean interrupted) -> {
          controlboard.driverResetRumble();
          ledController.setOverride(false);
          ledController.turnOffLeds(LedSections.ALL);
        })
        .unless(() -> {
          return drivetrain.isRedAlliance()
              ? (drivetrain.getPose().getX() <= Zones.LocationMath.COMMUNITY_ZONE_RED_MIN_X_M)
              : (drivetrain.getPose().getX() >= Zones.LocationMath.COMMUNITY_ZONE_BLUE_MAX_X_M);
        });
  }

  private static CommandBase prep(LedController ledController, Controlboard controlboard) {
    return prep(ledController, controlboard, LedColors.YELLOW);
  }

  private static CommandBase prep(
      LedController ledController, Controlboard controlboard, LedColors color
  ) {
    return Commands.sequence(ledController.setSolidColorCommand(color), new InstantCommand(() -> {
                               if (controlboard.getHeldGamePiece().isUnknown())
                                 controlboard.setHeldGamePiece(GamePieceType.CONE_GAME_PIECE);
                             }));
  }

  private static CommandBase confirmAndEject(
      Claw claw, LedController ledController, Controlboard controlboard
  ) {
    return Commands.sequence(
        Commands.waitUntil(() -> controlboard._xboxDrive.getLeftTriggerAxis() < 0.3),
        Commands.parallel(
            LedCommands.dynamicBlinkLedCommand(ledController, controlboard),
            Commands.race(
                new ClawOuttakeCommand(claw, controlboard),
                Commands.either(
                    Commands.waitSeconds(ScoringLevels.Consts.TELEOP_CUBE_SHOT_TIMEOUT_S),
                    Commands.waitSeconds(ScoringLevels.Consts.TELEOP_CUBE_SHOT_TIMEOUT_S),
                    () -> controlboard.getHeldGamePiece().isCube()
                )
            )
        )
    );
  }

  private static CommandBase driveAndSnap(
      Drivetrain drivetrain, Elevator elevator, Controlboard controlboard
  ) {
    return new TeleopSwerveCommand(
               drivetrain,
               elevator,
               controlboard,
               DriveModes.SNAP_TO_ANGLE,
               Constants.TRANSLATION_MAX_TRIM_SPEED_MPS,
               Constants.ANGLE_MAX_TRIM_SPEED_DPS
    )
        .beforeStarting(() -> drivetrain.setSnapScoringAngle());
  }

  private static CommandBase backoutAndStow(
      Claw claw, Drivetrain drivetrain, Elevator elevator, Controlboard controlboard
  ) {
    return Commands
        .deadline(
            Commands.sequence(
                Commands
                    .sequence(
                        claw.closeClawCommand(),
                        Commands.waitSeconds(ClawConfs.CLAW_DEPLOY_IN_TIME_S)
                    )
                    .unless(
                        () -> controlboard.getDesiredScoringLevel() == ScoringLevels.SCORING_LEVEL_0
                    ),
                StowCommands.tele(claw, controlboard, elevator)
            ),
            Commands.sequence(
                drivetrain.forceAllianceBasedFieldRelativeMovementCommand(1.0, 0.0, 0.3),
                new TeleopSwerveCommand(
                    drivetrain, elevator, controlboard, DriveModes.FIELD_RELATIVE
                )
            )
        )
        .finallyDo((boolean interrupted) -> {
          if (!interrupted) {
            controlboard.setHeldGamePiece(GamePieceType.UNKNOWN_GAME_PIECE);
          }
        });
  }
}
