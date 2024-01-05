package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

public class GoToPoseCommand extends CommandBase {
  private final Drivetrain _drivetrain;
  private final Transform2d _relative;
  private final boolean _isRelative;
  private final double _xToleranceM;
  private final double _yToleranceM;
  private final Rotation2d _thetaTolerance;
  private Pose2d _target;

  public GoToPoseCommand(
      Drivetrain drivetrain,
      Pose2d pose,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    _isRelative = false;
    _drivetrain = drivetrain;
    _xToleranceM = xToleranceM;
    _yToleranceM = yToleranceM;
    _thetaTolerance = thetaTolerance;

    if (_isRelative) {
      _relative = new Transform2d(new Pose2d(), pose);
    } else {
      _target = pose;
      _relative = null;
    }

    addRequirements(drivetrain);
  }

  public GoToPoseCommand(
      Drivetrain drivetrain,
      Transform2d relative,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    _isRelative = true;
    _drivetrain = drivetrain;
    _relative = relative;
    _xToleranceM = xToleranceM;
    _yToleranceM = yToleranceM;
    _thetaTolerance = thetaTolerance;

    addRequirements(drivetrain);
  }

  public GoToPoseCommand(Drivetrain drivetrain, Pose2d pose) {
    this(
        drivetrain,
        pose,
        Constants.CRobot.drive.control.xy.toleranceM,
        Constants.CRobot.drive.control.xy.toleranceM,
        Constants.CRobot.drive.control.theta.tolerance
    );
  }

  @Override
  public void initialize() {
    _drivetrain.setTolerance(_xToleranceM, _yToleranceM, _thetaTolerance);

    if (_isRelative) {
      _target = _drivetrain.getPose().transformBy(_relative);
    }

    _drivetrain.setStaticTarget(_target);
  }

  @Override
  public void execute() {
    _drivetrain.chaseStaticTargetDrive();
  }

  @Override
  public void end(boolean interrupted) {
    _drivetrain.resetDefaultTolerance();
    _drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return _drivetrain.inRange();
  }
}
