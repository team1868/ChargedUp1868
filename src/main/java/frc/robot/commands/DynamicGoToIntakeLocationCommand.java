package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.CameraSets;
import frc.robot.constants.enums.HPIntakeStations;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.MultiCameraController;

public class DynamicGoToIntakeLocationCommand extends CommandBase {
  private final Drivetrain _drivetrain;
  private final Controlboard _controlboard;
  private final Transform2d _redOffset;
  private final Transform2d _blueOffset;
  private final HPIntakeStations _intakeLocation;
  private final double _xToleranceM;
  private final double _yToleranceM;
  private Rotation2d _thetaTolerance;

  public DynamicGoToIntakeLocationCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      Translation2d redOffset,
      HPIntakeStations intakeLocation,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    _drivetrain = drivetrain;
    _controlboard = controlboard;
    _redOffset = new Transform2d(redOffset, Rotation2d.fromDegrees(0.0));
    _intakeLocation = intakeLocation;
    _xToleranceM = xToleranceM;
    _yToleranceM = yToleranceM;
    _thetaTolerance = thetaTolerance;
    _blueOffset = _redOffset.inverse();

    addRequirements(drivetrain);
  }

  public DynamicGoToIntakeLocationCommand(Drivetrain drivetrain, Controlboard controlboard) {
    this(drivetrain, controlboard, new Translation2d(0, 0));
  }

  public DynamicGoToIntakeLocationCommand(
      Drivetrain drivetrain, Controlboard controlboard, Translation2d redOffset
  ) {
    this(drivetrain, controlboard, redOffset, HPIntakeStations.UNKNOWN_INTAKE_STATION);
  }

  public DynamicGoToIntakeLocationCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      Translation2d redOffset,
      HPIntakeStations intakeLocation
  ) {
    this(
        drivetrain,
        controlboard,
        redOffset,
        intakeLocation,
        Constants.CRobot.drive.control.xy.toleranceM,
        Constants.CRobot.drive.control.xy.toleranceM
    );
  }

  public DynamicGoToIntakeLocationCommand(
      Drivetrain drivetrain, Controlboard controlboard, Translation2d redOffset, double xyTolerance
  ) {
    this(drivetrain, controlboard, redOffset, HPIntakeStations.UNKNOWN_INTAKE_STATION, xyTolerance);
  }

  public DynamicGoToIntakeLocationCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      Translation2d redOffset,
      HPIntakeStations intakeLocation,
      double xyTolerance
  ) {
    this(drivetrain, controlboard, redOffset, intakeLocation, xyTolerance, xyTolerance);
  }

  public DynamicGoToIntakeLocationCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      Translation2d redOffset,
      double xTolerance,
      double yTolerance
  ) {
    this(
        drivetrain,
        controlboard,
        redOffset,
        HPIntakeStations.UNKNOWN_INTAKE_STATION,
        xTolerance,
        yTolerance
    );
  }

  public DynamicGoToIntakeLocationCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      Translation2d redOffset,
      HPIntakeStations intakeLocation,
      double xTolerance,
      double yTolerance
  ) {
    this(
        drivetrain,
        controlboard,
        redOffset,
        intakeLocation,
        xTolerance,
        yTolerance,
        Constants.CRobot.drive.control.theta.tolerance
    );
  }

  public DynamicGoToIntakeLocationCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      Translation2d redOffset,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    this(
        drivetrain,
        controlboard,
        redOffset,
        HPIntakeStations.UNKNOWN_INTAKE_STATION,
        xToleranceM,
        yToleranceM,
        thetaTolerance
    );
  }

  @Override
  public void initialize() {
    boolean isRedAlliance = _drivetrain.isRedAlliance();
    Pose2d pose = _intakeLocation == HPIntakeStations.UNKNOWN_INTAKE_STATION
        ? _controlboard.getDesiredIntakeLocation().pose.get(isRedAlliance)
        : _intakeLocation.pose.get(isRedAlliance);
    Pose2d offsetPose = pose.plus(isRedAlliance ? _redOffset : _blueOffset);

    if (_intakeLocation == HPIntakeStations.DOUBLE_STATION_INNER) { // inner
      _drivetrain.setDesiredCameras(isRedAlliance ? CameraSets.CAMERA_1 : CameraSets.CAMERA_0);
    } else {
      _drivetrain.setDesiredCameras(isRedAlliance ? CameraSets.CAMERA_0 : CameraSets.CAMERA_1);
    }

    _drivetrain.setTolerance(_xToleranceM, _yToleranceM, _thetaTolerance);
    _drivetrain.setStaticTarget(offsetPose);
    _drivetrain.chaseStaticTargetDrive();
  }

  @Override
  public void execute() {
    _drivetrain.chaseStaticTargetDrive();
  }

  @Override
  public void end(boolean interrupted) {
    _drivetrain.resetDefaultTolerance();
    _drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
    _drivetrain.setDesiredCameras(Constants.CRobot.vision.setup);
  }

  @Override
  public boolean isFinished() {
    return _drivetrain.inRange();
    // TODO: fill in with actual value;
  }
}
