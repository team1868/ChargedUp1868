package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class TeleopSwerveCommand extends CommandBase {
  private final Drivetrain _drivetrain;
  private final Elevator _elevator;
  private final Controlboard _controlboard;
  private DriveModes _driveMode;
  private double _maxSpeedMPS;
  private Rotation2d _maxAngularSpeedPS;

  private static final DriveModes DEFAULT_DRIVE_MODE = DriveModes.FIELD_RELATIVE;

  public TeleopSwerveCommand(
      Drivetrain drivetrain,
      Elevator elevator,
      Controlboard controlboard,
      DriveModes initialMode,
      double maxSpeedMPS,
      Rotation2d maxAngularSpeedPS
  ) {
    _drivetrain = drivetrain;
    _elevator = elevator;
    _controlboard = controlboard;
    _driveMode = initialMode;
    _maxSpeedMPS = maxSpeedMPS;
    _maxAngularSpeedPS = maxAngularSpeedPS;

    addRequirements(drivetrain);
  }

  public TeleopSwerveCommand(
      Drivetrain drivetrain, Elevator elevator, Controlboard controlboard, DriveModes initialMode
  ) {
    this(
        drivetrain,
        elevator,
        controlboard,
        initialMode,
        Constants.CRobot.drive.control.defaultLimits.maxTranslationalVelocityMPS,
        Constants.CRobot.drive.control.defaultLimits.maxAngularVelocityPS
    );
  }

  public void teleopInit() {
    _driveMode = DEFAULT_DRIVE_MODE;
  }

  public boolean condActivateSnapToAngle(int anglePOV) {
    if (anglePOV != -1) {
      _driveMode = DriveModes.SNAP_TO_ANGLE;
      // note that the pov axis does not align, we'll need some sort of flat
      // offset here Our functional 0 will be right facing to align with
      // odometry (3 on a standard clock face) POV 0 is "forward" (12 on a
      // standard clock face)
      // _drivetrain.setSnapAngle(((double) -anglePOV) + _drivetrain.POVZeroOffsetDeg()); // odom
      // zero offset

      var deg = ((double) -anglePOV) + _drivetrain.POVZeroOffsetDeg();
      if (Math.abs(anglePOV) == 180.0 || Math.abs(anglePOV) == 0.0 || Math.abs(anglePOV) == 90.0
          || Math.abs(anglePOV) == 270.0) {
        _drivetrain.setSnapAngleDeg(deg);
      }
      return true;
    }
    return false;
  }

  public void condActivateSnapToAngleCritical(boolean trigger) {
    if (trigger) {
      _driveMode = DriveModes.SNAP_TO_ANGLE;

      _drivetrain.setSnapAngleDeg(
          _drivetrain.isRedAlliance()
                  ^ (_drivetrain.getPose().getX() > Constants.CField.dims.halfX_M)
              ? 180.0
              : 0.0
      );
    }
  }

  public boolean condActivateChaseStaticTarget(Pose2d pose) {
    if (_controlboard._xboxDrive.getHID().getXButtonPressed()) {
      _driveMode = DriveModes.CHASE_STATIC_TARGET;
      _drivetrain.setStaticTarget(pose);
      return true;
    }
    return false;
  }

  public boolean condActivateSnake() {
    if (_controlboard._xboxDrive.getHID().getLeftBumperPressed()) {
      _driveMode = DriveModes.SNAKE;
      return true;
    }
    return false;
  }

  public void setAntiTipMode() {
    _driveMode = DriveModes.FIELD_RELATIVE_ANTI_TIP;
  }

  public void setDefaultDriveMode() {
    _driveMode = DEFAULT_DRIVE_MODE;
  }

  @Override
  public void execute() {
    // this code should probably move, but here before the state machine
    // management is also ok while dpad is pressed, set drive mode to
    // SNAP_TO_ANGLE and give the drivetrain a new target angle
    int anglePOV = 0; // needs to be initialized, but it doesn't matter what it's initalized to
                      // right? bc anglePOV isn't used unless isPIDTuningMode

    double xAxis = _controlboard.getDriveX();
    double yAxis = _controlboard.getDriveY();
    double rAxis = _controlboard.getRotX();
    double xyNet = Math.hypot(xAxis, yAxis);

    if (RobotAltModes.isPIDTuningMode) {
      anglePOV = _controlboard._xboxDrive.getHID().getPOV();
    }
    switch (_driveMode) {
      case ROBOT_CENTRIC:
      case FIELD_RELATIVE:
      case FIELD_RELATIVE_ROTATION_COMPENSATION_SCALING:
      case SLEWING_FIELD_RELATIVE:
      case FIELD_RELATIVE_SKEW_COMPENSATION:
      case FIELD_RELATIVE_254:
        condActivateSnapToAngle(_controlboard._xboxDrive.getHID().getPOV());
        // condActivateSnapToAngleCritical(_controlboard._xboxDrive.getHID().getAButtonPressed());

        if (RobotAltModes.isPIDTuningMode && anglePOV != -1) {
          _drivetrain.testSteer((double) anglePOV);
        }
        break;
      case FIELD_RELATIVE_ANTI_TIP:
        _drivetrain.drive(
            xAxis,
            yAxis,
            xyNet,
            rAxis,
            _elevator.getCurrentHeight() > ElevatorPositions.ELEVATOR_TIP_PREVENTION_THRESHOLD.ticks
                ? DriveModes.FIELD_RELATIVE_ANTI_TIP
                : DEFAULT_DRIVE_MODE,
            _maxSpeedMPS
        );
        return;
      case SNAP_TO_ANGLE:
        if (rAxis > Control.STICK_DEADBAND || rAxis < -Control.STICK_DEADBAND) {
          _driveMode = DEFAULT_DRIVE_MODE;
        } else {
          condActivateSnapToAngle(_controlboard._xboxDrive.getHID().getPOV());
        }
        break;
      case SNAKE:
        if (_controlboard._xboxDrive.getHID().getLeftBumperPressed()
            || rAxis > Control.STICK_DEADBAND || rAxis < -Control.STICK_DEADBAND) {
          _driveMode = DEFAULT_DRIVE_MODE;
        }
        break;
      case TARGET_RELATIVE:
      case CHASE_STATIC_TARGET:
        if (rAxis > Control.STICK_DEADBAND || rAxis < -Control.STICK_DEADBAND
            || xyNet > Control.STICK_NET_DEADBAND
            || _controlboard._xboxDrive.getHID().getXButtonPressed()) {
          _driveMode = DEFAULT_DRIVE_MODE;
        }
        break;
      case CHASE_DYNAMIC_TARGET:
      default:
        System.err.println("Unkown drive mode");
        _driveMode = DEFAULT_DRIVE_MODE;
        break;
    }

    // We need logic here or in the drive commands that allows for desaturation to occur
    // contextually, capping maximal turning speed when stationary but not allowing the drive
    // command to override it due to maximum rotational speed being capped lower relative to the
    // theoretical value
    _drivetrain.drive(xAxis, yAxis, xyNet, rAxis, _driveMode, _maxSpeedMPS, _maxAngularSpeedPS);
  }
}
