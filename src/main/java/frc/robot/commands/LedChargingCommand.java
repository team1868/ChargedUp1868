package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.LedConfs;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.LedModes;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LedController;

public class LedChargingCommand extends CommandBase {
  private final LedController _ledController;
  private final Drivetrain _drivetrain;
  private final Controlboard _controlboard;
  private boolean _redAlliance;
  private boolean _facingSubstationRed;
  private boolean _useRoll;
  private double _currAngle;

  public LedChargingCommand(
      LedController ledController, Drivetrain drivetrain, Controlboard controlboard
  ) {
    _ledController = ledController;
    _drivetrain = drivetrain;
    _controlboard = controlboard;

    addRequirements(ledController);
  }

  @Override
  public void initialize() {
    _redAlliance = _drivetrain.isRedAlliance();
  }

  @Override
  public void execute() {
    calculateRobotPosition();

    // robot closer to alliance: blue LEDs, closer to center: yellow LEDs
    if (_currAngle < -LedConfs.LEVEL_THRESHOLD) {
      boolean temp = _facingSubstationRed ? _redAlliance : !_redAlliance;
      _ledController.setSolidColor(temp ? LedColors.YELLOW : LedColors.BLUE, LedSections.ALL);
    } else if (_currAngle > LedConfs.LEVEL_THRESHOLD) {
      boolean temp = _facingSubstationRed ? !_redAlliance : _redAlliance;
      _ledController.setSolidColor(temp ? LedColors.YELLOW : LedColors.BLUE, LedSections.ALL);
    } else {
      _ledController.changeAnimation(LedModes.RAINBOW);
    }
  }

  @Override
  public void end(boolean interrupted) {
    _ledController.turnOffLeds(LedSections.ALL);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void calculateRobotPosition() {
    double yaw = _drivetrain.getYaw().getDegrees() % 360.0;
    yaw = yaw + yaw < 0.0 ? 360.0 : 0.0;
    _useRoll = (yaw >= LedConfs.ROLL_1_LOWER_DEG && yaw <= LedConfs.ROLL_1_UPPER_DEG)
        || (yaw >= LedConfs.ROLL_2_LOWER_DEG && yaw <= LedConfs.ROLL_2_UPPER_DEG);
    _facingSubstationRed =
        ((yaw >= LedConfs.ORIENTATION_BLUE_LOWER_DEG && yaw <= LedConfs.ORIENTATION_BLUE_UPPER_DEG)
         && !_redAlliance)
        || ((yaw >= LedConfs.ORIENTATION_RED_LOWER_DEG || yaw <= LedConfs.ORIENTATION_RED_UPPER_DEG)
            && _redAlliance);
    _currAngle =
        _useRoll ? _drivetrain.getRoll().getDegrees() : _drivetrain.getPitch().getDegrees();
  }
}
