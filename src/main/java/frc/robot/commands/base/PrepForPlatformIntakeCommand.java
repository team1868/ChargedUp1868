package frc.robot.commands.base;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.CommandUtils.ElevatorUtils;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Elevator;

public class PrepForPlatformIntakeCommand extends ParallelDeadlineGroup {
  public PrepForPlatformIntakeCommand(Claw claw, Controlboard controlboard, Elevator elevator) {
    super(
        ElevatorUtils.pieceSpecificElevatorRaiseToCommand(
            claw,
            controlboard,
            elevator,
            ElevatorPositions.ELEVATOR_PLATFORM_CUBE_INTAKE,
            ElevatorPositions.ELEVATOR_PLATFORM_CONE_INTAKE
        ),
        new ClawOpenToCommand(claw, ClawPositions.CLAW_OPEN_POS_TICKS)
    );
  }
}
