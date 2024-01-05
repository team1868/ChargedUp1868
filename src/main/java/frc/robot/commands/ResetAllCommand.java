package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.enums.DriveModes;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class ResetAllCommand extends CommandBase {
  private final Claw _claw;
  private final Drivetrain _drivetrain;
  private final Elevator _elevator;

  public ResetAllCommand(Claw claw, Drivetrain drivetrain, Elevator elevator) {
    _drivetrain = drivetrain;
    _claw = claw;
    _elevator = elevator;

    addRequirements(drivetrain, claw, elevator);
  }

  @Override
  public void initialize() {
    _drivetrain.drive(0.0, 0.0, 0.0, DriveModes.FIELD_RELATIVE);

    _claw.setClawRollersPower(0.0);
    // _claw.setClawPower(0.0) for motorClaw
    _claw.closeClawSolenoid();

    _elevator.setPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
