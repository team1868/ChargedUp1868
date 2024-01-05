package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.subsystems.*;

public class StowCommands {
  public static CommandBase tele(Claw claw, Controlboard controlboard, Elevator elevator) {
    return new ElevatorRaiseToCommand(elevator, controlboard, ElevatorPositions.ELEVATOR_STOW_TICKS)
        .finallyDo((boolean interrupted) -> claw.closeClawSolenoid());
  }

  public static CommandBase auto(Claw claw, Controlboard controlboard, Elevator elevator) {
    return Commands.sequence(
        new ElevatorRaiseToCommand(elevator, controlboard, ElevatorPositions.ELEVATOR_STOW_TICKS),
        claw.closeClawCommand()
    );
  }
}
