package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.CameraSets;
import frc.robot.constants.enums.ScoringLocations;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;

public class DynamicGoToScoringLocationCommand extends CommandBase {
  private final Drivetrain _drivetrain;
  private final Controlboard _controlboard;
  private final boolean _forceWheel;
  private final Transform2d _redOffset;
  private final Transform2d _blueOffset;
  private final ScoringLocations _scoringLocation;
  private final double _xToleranceM;
  private final double _yToleranceM;
  private final Rotation2d _thetaTolerance;
  private Pose2d _pose;
  private boolean _thetaAlignedDeg;
  private double _moduleAngleDeg;

  public DynamicGoToScoringLocationCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      boolean forceWheel,
      Transform2d redOffset,
      ScoringLocations scoringLocation,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    _drivetrain = drivetrain;
    _controlboard = controlboard;
    _forceWheel = forceWheel;
    _redOffset = redOffset;
    _blueOffset = _redOffset.inverse();
    _scoringLocation = scoringLocation;
    _xToleranceM = xToleranceM;
    _yToleranceM = yToleranceM;
    _thetaTolerance = thetaTolerance;

    addRequirements(drivetrain);
  }

  public DynamicGoToScoringLocationCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      boolean forceWheel,
      Transform2d redOffset,
      ScoringLocations scoringLocation
  ) {
    this(
        drivetrain,
        controlboard,
        forceWheel,
        redOffset,
        scoringLocation,
        Constants.CRobot.drive.control.xy.toleranceM,
        Constants.CRobot.drive.control.xy.toleranceM,
        Constants.CRobot.drive.control.theta.tolerance
    );
  }

  @Override
  public void initialize() {
    _controlboard.driverRumble();

    boolean isRed = _drivetrain.isRedAlliance();
    ScoringLocations location = _scoringLocation == ScoringLocations.UNKNOWN_SCORING_LOCATION
        ? _controlboard.getDesiredScoringLocation()
        : _scoringLocation;
    _pose = location.pose.get(isRed);

    _drivetrain.setDesiredCameras(location.getScoringCamera(isRed));

    _drivetrain.setTolerance(_xToleranceM, _yToleranceM, _thetaTolerance);
    _drivetrain.setStaticTarget(_pose);

    if (_forceWheel) {
      _thetaAlignedDeg = false;
      _drivetrain.setSnapAngle(_pose.getRotation());
      _drivetrain.snapToAngleDrive(0.0, 0.0);
    } else {
      _drivetrain.chaseStaticTargetDrive();
    }
  }

  @Override
  public void execute() {
    if (_forceWheel) {
      if (!_thetaAlignedDeg) {
        if (_drivetrain.thetaInRange()) {
          _thetaAlignedDeg = true;
          var curr = _drivetrain.getPose();
          _moduleAngleDeg = Math.atan2(_pose.getX() - curr.getX(), _pose.getY() - curr.getY());
          _drivetrain.forceScoreWheelDirection(_moduleAngleDeg);
        } else {
          _drivetrain.snapToAngleDrive(0.0, 0.0);
        }
      } else {
        _drivetrain.forceWheelDirectionDrive(
            _moduleAngleDeg, Constants.FORCED_WHEEL_ALIGNMENT_SPEED_MPS
        );
      }
    } else {
      _drivetrain.chaseStaticTargetDrive();
    }
  }

  @Override
  public void end(boolean interrupted) {
    _drivetrain.resetDefaultTolerance();
    _drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
    _drivetrain.setDesiredCameras(CameraSets.BOTH_CAMERAS);
    _controlboard.driverResetRumble();
  }

  @Override
  public boolean isFinished() {
    return _drivetrain.inRange();
  }
}
