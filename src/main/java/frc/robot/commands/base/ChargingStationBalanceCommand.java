package frc.robot.commands.base;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import java.lang.Math;

public class ChargingStationBalanceCommand extends CommandBase {
  private Drivetrain _drivetrain;
  private Controlboard _controlboard;
  private boolean _allianceInvert;
  private double _tiltDeg;
  private double _targetRangeDeg;
  private double _trimRangeDeg;
  private double _trimSpeedMPS;
  private double _maxSpeedMPS;
  private double _desiredSpeedMPS;
  private boolean _usingPitch;
  private boolean _negateTilt;
  private Timer _timer = new Timer();

  public ChargingStationBalanceCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      boolean allianceInvert,
      double targetRangeDeg,
      double trimRangeDeg,
      double trimSpeedMPS,
      double maxSpeedMPS
  ) {
    _drivetrain = drivetrain;
    _controlboard = controlboard;
    _allianceInvert = allianceInvert;
    _targetRangeDeg = Math.abs(targetRangeDeg);
    _trimRangeDeg = Math.abs(trimRangeDeg);
    _trimSpeedMPS = trimSpeedMPS;
    _maxSpeedMPS = maxSpeedMPS;

    addRequirements(drivetrain);
  }

  public ChargingStationBalanceCommand(Drivetrain drivetrain, Controlboard controlboard) {
    this(drivetrain, controlboard, false, 7, 16, 0.13, 0.35);
  }

  @Override
  public void initialize() {
    configureAndSnapAngle();

    _timer.restart();
  }

  @Override
  public void execute() {
    configureAndSnapAngle();

    double _tilt =
        _usingPitch ? _drivetrain.getPitch().getDegrees() : _drivetrain.getRoll().getDegrees();
    _tilt = _tilt * (_negateTilt ? 1.0 : -1.0);
    double absTilt = Math.abs(_tilt);

    if (absTilt > _targetRangeDeg) {
      _timer.reset();
      _desiredSpeedMPS = absTilt > _trimRangeDeg ? _maxSpeedMPS : _trimSpeedMPS;
      _desiredSpeedMPS = _tilt <= 0 ? -_desiredSpeedMPS : _desiredSpeedMPS;
    } else {
      _timer.start();
      _desiredSpeedMPS = 0;
    }

    if (_allianceInvert) {
      _drivetrain.snapToAngleDrive(
          DriverStation.getAlliance() == DriverStation.Alliance.Red ? _desiredSpeedMPS
                                                                    : -_desiredSpeedMPS,
          0
      );
    } else {
      _drivetrain.snapToAngleDrive(_desiredSpeedMPS, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    _drivetrain.forceChargingWheelDirection();
    _drivetrain.fieldRelativeDrive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return _timer.hasElapsed(1.0);
    // TODO: fill in with actual value;
  }

  public void configureAndSnapAngle() {
    double yaw = _drivetrain.toAbsoluteAngleDeg(_drivetrain.getYaw().getDegrees());

    if (yaw > 315 || yaw < 45) {
      _drivetrain.setSnapAngleDeg(0);
      _usingPitch = true;
      _negateTilt = false;
    } else if (yaw >= 45 && yaw < 135) {
      _drivetrain.setSnapAngleDeg(90);
      _usingPitch = false;
      _negateTilt = false;

    } else if (yaw >= 135 && yaw < 225) {
      _drivetrain.setSnapAngleDeg(180);
      _usingPitch = true;
      _negateTilt = true;
    } else {
      // } else if (yaw >= 225_deg || yaw < 315_deg) {
      _drivetrain.setSnapAngleDeg(270);
      _usingPitch = false;
      _negateTilt = true;
    }
  }
}
