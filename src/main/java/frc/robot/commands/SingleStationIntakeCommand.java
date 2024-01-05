package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.base.ClawOpenToCommand;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.ClawConfs.IntakeSpeed;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.constants.enums.ClawDirections;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.constants.enums.HPIntakeStations;
import frc.robot.subsystems.*;

public class SingleStationIntakeCommand {
  public static CommandBase dynamic(
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
                    new ClawOpenToCommand(claw, ClawPositions.CLAW_CLOSED_POS_TICKS),
                    ClawRollersCommands.desiredPieceCommand(
                        claw,
                        controlboard,
                        IntakeSpeed.CUBE_POWER,
                        ClawDirections.CLAW_ROLLERS_CUBE_IN,
                        IntakeSpeed.CONE_POWER,
                        ClawDirections.CLAW_ROLLERS_CONE_IN
                    ),
                    ClawRollersCommands.waitSequenceCommand(claw, controlboard)
                ),
                new DynamicGoToIntakeLocationCommand(
                    drivetrain,
                    controlboard,
                    new Translation2d(0.0, 0.0),
                    HPIntakeStations.SINGLE_STATION,
                    Units.inchesToMeters(2),
                    Units.inchesToMeters(9),
                    Rotation2d.fromDegrees(1.0)
                ),
                new ElevatorRaiseToCommand(
                    elevator, controlboard, ElevatorPositions.ELEVATOR_DROP_CONE_INTAKE
                )
            ),
            ClawRollersCommands.desiredPieceStallCommand(claw, controlboard),
            StowCommands.tele(claw, controlboard, elevator)
        )
        .finallyDo((boolean interrupted) -> {
          if (interrupted)
            claw.setClawRollersPower(ClawConfs.CLAW_CONE_STALL_POWER);
        });
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
                    new ClawOpenToCommand(claw, ClawPositions.CLAW_CLOSED_POS_TICKS),
                    ClawRollersCommands.desiredPieceCommand(
                        claw,
                        controlboard,
                        IntakeSpeed.CUBE_POWER,
                        ClawDirections.CLAW_ROLLERS_CUBE_IN,
                        IntakeSpeed.CONE_POWER,
                        ClawDirections.CLAW_ROLLERS_CONE_IN
                    ),
                    ClawRollersCommands.waitSequenceCommand(claw, controlboard)
                ),
                LedCommands.dynamicBlinkLedCommand(ledController, controlboard),
                new ElevatorRaiseToCommand(
                    elevator, controlboard, ElevatorPositions.ELEVATOR_DROP_CONE_INTAKE
                )
            ),
            ClawRollersCommands.desiredPieceStallCommand(claw, controlboard),
            StowCommands.tele(claw, controlboard, elevator)
        )
        .alongWith(
            new TeleopSwerveCommand(drivetrain, elevator, controlboard, DriveModes.SNAP_TO_ANGLE)
                .beforeStarting(() -> drivetrain.setSnapAngleDeg(90.0))
        )
        .beforeStarting(() -> {
          ledController.setOverride(true);
          ledController.turnOffLeds(LedSections.ALL);
        })
        .finallyDo((boolean interrupted) -> {
          if (interrupted)
            claw.setClawRollersPower(ClawConfs.CLAW_CONE_STALL_POWER);
          ledController.setOverride(false);
          ledController.turnOffLeds(LedSections.ALL);
        });
  }
}
