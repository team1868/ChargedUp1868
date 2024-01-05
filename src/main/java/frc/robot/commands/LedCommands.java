package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.constants.enums.LedModes;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.LedController;

// Raising elevator: blue
// Raising intake: pink
// Intake rollers: purple
// Retract intake: green

public class LedCommands {
  public static CommandBase dynamicBlinkLedCommand(
      LedController ledController, Controlboard controlboard
  ) {
    return Commands.sequence(
        new InstantCommand(() -> ledController.turnOffLeds(LedSections.ALL)),
        Commands.either(
            ledController.changeAnimationCommand(LedModes.BLINK_CUBE),
            ledController.changeAnimationCommand(LedModes.BLINK_CONE),
            () -> controlboard.getDesiredGamePiece().isCube()
        )
    );
  }
}
