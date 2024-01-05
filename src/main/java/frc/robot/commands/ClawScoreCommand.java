package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.enums.ClawScoreLevels;
import frc.robot.subsystems.Claw;

public class ClawScoreCommand extends SequentialCommandGroup {
  private final Claw _claw;
  private final ClawScoreLevels _scoringLevel;
  private final double _scoringDuration;
  public ClawScoreCommand(Claw claw, double scoringDuration, ClawScoreLevels scoringLevel) {
    _claw = claw;
    _scoringDuration = scoringDuration;
    _scoringLevel = scoringLevel;

    addCommands(_claw.openClawCommand()
                    .andThen(new WaitUntilCommand(_claw::isExtended))
                    .andThen(_claw.setClawPowerCommand(_scoringLevel.power))
                    .andThen(new WaitCommand(_scoringDuration))
                    .andThen(_claw.clawRollersOffCommand().alongWith(
                        _claw.closeClawCommand().alongWith(new WaitUntilCommand(_claw::isRetracted))
                    )));
  }
}
