package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.PrepForPlatformIntakeCommand;
import frc.robot.commands.base.SetClawGivenElevatorCommand;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.Constants;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.HPIntakeStations;
import frc.robot.constants.enums.LedColors;
import frc.robot.subsystems.*;

public class PlatformIntakeCommand {
  public static CommandBase dynamic(
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard,
      Pose2d platformPose
  ) {
    return Commands.sequence(
        Commands.parallel(
            ledController.setSolidColorCommand(LedColors.PURPLE),
            new DynamicGoToIntakeLocationCommand(
                drivetrain,
                controlboard,
                new Translation2d(Constants.PLATFORM_PREP_DIST_M, 0.0),
                HPIntakeStations.UNKNOWN_INTAKE_STATION,
                Units.inchesToMeters(5.0)
            )
        ),
        // Potentially parallelize with go previous got to intake location
        new PrepForPlatformIntakeCommand(claw, controlboard, elevator),

        // Allow the driver to reposition as long as claw beam break not triggered
        Commands.race(
            // Integrate intake retraction if using partial stow
            Commands
                .sequence(
                    Commands.either(
                        new DynamicGoToIntakeLocationCommand(drivetrain, controlboard),
                        new DynamicGoToIntakeLocationCommand(
                            drivetrain,
                            controlboard,
                            new Translation2d(Constants.PLATFORM_INTAKE_CONE_OFFSET_M, 0.0)
                        ),
                        () -> controlboard.getDesiredGamePiece().isCube()
                    ),
                    // Turn rumble off while driver has control
                    new TeleopSwerveCommand(
                        drivetrain,
                        elevator,
                        controlboard,
                        DriveModes.FIELD_RELATIVE,
                        Constants.TRANSLATION_MAX_TRIM_SPEED_MPS,
                        Constants.ANGLE_MAX_TRIM_SPEED_DPS
                    )
                        .beforeStarting(controlboard.driverResetRumbleCommand())
                )
                .finallyDo((boolean interrupted) -> controlboard.driverRumble()),
            ClawIntakeCommand.ClawIntake(claw, controlboard, ledController)
        ),
        Commands.deadline(
            Commands.waitSeconds(ClawConfs.CLAW_PLATFORM_CLEARANCE_WAIT_S)
                .andThen(Commands.parallel(
                    new SetClawGivenElevatorCommand(
                        claw,
                        controlboard,
                        elevator,
                        ClawPositions.CLAW_MIN_TICKS,
                        ElevatorPositions.ELEVATOR_LEVEL_2_CUBE.ticks,
                        false
                    ),
                    StowCommands.tele(claw, controlboard, elevator)
                )),
            Commands.sequence(
                new DynamicGoToIntakeLocationCommand(
                    drivetrain,
                    controlboard,
                    new Translation2d(Constants.PLATFORM_BACKOUT_DIST_M, 0.0),
                    HPIntakeStations.UNKNOWN_INTAKE_STATION,
                    Units.inchesToMeters(10.0),
                    Units.inchesToMeters(10.0),
                    Rotation2d.fromDegrees(20.0)
                )
                    .finallyDo((boolean interrupted) -> controlboard.driverResetRumble())
                    .withTimeout(2.0),
                new TeleopSwerveCommand(
                    drivetrain,
                    elevator,
                    controlboard,
                    DriveModes.FIELD_RELATIVE,
                    Constants.TRANSLATION_MAX_TRIM_SPEED_MPS,
                    Constants.ANGLE_MAX_TRIM_SPEED_DPS
                )
            )
        )
        // ,
        //         #ifdef NEW_CLAW
        //                DesiredPieceStallRollers(claw, controlboard))
        // #else
        //                ClawRollersOnCommand(claw, CLAW_ROLLERS_IN,
        //                CLAW_CONE_STALL_POWER).ToPtr())
        // #endif
        //         .BeforeStarting([&]() {
        //             controlboard.DriverRumble();
        //             ledController.SetOverride(true);
        //         })
        //         .FinallyDo([&](bool interrupted) {
        //             controlboard.DriverResetRumble();
        //             ledController.SetOverride(false);
        //             ledController.TurnOffLeds(ALL);
        //         })
        //         .Unless([&]() {
        //             auto pose = drivetrain.GetPose();
        //             auto inRange =
        //                 (pose.Y() > HALF_FIELD_Y && (drivetrain.GetAlliance() ==
        //                 frc::DriverStation::kRed
        //                                                  ? (pose.X() < LOADING_ZONE_RED_MAX_X)
        //                                                  : (pose.X() >
        //                                                  LOADING_ZONE_BLUE_MIN_X)));
        //             if (!inRange) { ledController.SetSolidColor(RED_ORANGE, ALL); }
        //             return !inRange;
        //         })
    );
  }

  public static CommandBase manual(
      Claw claw,
      Drivetrain drivetrain,
      Elevator elevator,
      LedController ledController,
      Controlboard controlboard
  ) {
    return Commands
        .sequence(
            Commands.deadline(
                Commands.sequence(
                    Commands.either(
                        ledController.setSolidColorCommand(LedColors.PURPLE),
                        ledController.setSolidColorCommand(LedColors.YELLOW),
                        () -> controlboard.getDesiredGamePiece().isCube()
                    ),
                    new PrepForPlatformIntakeCommand(claw, controlboard, elevator),
                    ClawIntakeCommand.ClawIntake(claw, controlboard, ledController)
                ),
                new TeleopSwerveCommand(
                    drivetrain,
                    elevator,
                    controlboard,
                    DriveModes.SNAP_TO_ANGLE,
                    Constants.TRANSLATION_MAX_TRIM_SPEED_MPS,
                    Constants.ANGLE_MAX_TRIM_SPEED_DPS
                )
                    .beforeStarting(() -> {
                      drivetrain.setSnapAngle(
                          drivetrain.isRedAlliance()
                              ? HPIntakeStations.LocationMath.RED_PLATFORM_ANGLE
                              : HPIntakeStations.LocationMath.BLUE_PLATFORM_ANGLE
                      );
                    })
            ),
            drivetrain
                .forceAllianceBasedFieldRelativeMovementCommand(
                    -2.0, 0.0, Units.millisecondsToSeconds(200)
                )
                .beforeStarting(claw.closeClawCommand()),
            Commands.deadline(
                StowCommands.tele(claw, controlboard, elevator),
                new TeleopSwerveCommand(
                    drivetrain, elevator, controlboard, DriveModes.FIELD_RELATIVE
                )
            )
        )
        .finallyDo((boolean interrupted) -> { ledController.turnOffLeds(LedSections.ALL); });
  }
}
