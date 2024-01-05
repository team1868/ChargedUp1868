// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.commands.base.ClawOpenToCommand;
import frc.robot.commands.base.ClawOuttakeCommand;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.ElevatorZeroCommand;
import frc.robot.commands.base.GoToPoseCommand;
import frc.robot.commands.score.TeleopScoreCommands;
import frc.robot.constants.ClawConfs;
import frc.robot.constants.Constants;
import frc.robot.constants.LedConfs;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.AutoCommand;
import frc.robot.constants.enums.AutonomousAction;
import frc.robot.constants.enums.AutonomousRoutines;
import frc.robot.constants.enums.AutonomousRoutines.Builders;
import frc.robot.constants.enums.ClawDirections;
import frc.robot.constants.enums.ClawPositions;
import frc.robot.constants.enums.ClawScoreLevels;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.ElevatorPositions;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.LedModes;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.constants.enums.ScoringLocations;
import frc.robot.subsystems.*;
import frc.robot.utils.LoopTimer;
import java.util.Map;

public class RobotContainer {
  /* --- Shared Resources --- */
  private Field2d _field = new Field2d();

  /* --- Subsystems --- */
  public Controlboard _controlboard = new Controlboard(_field);
  public Drivetrain _drivetrain = new Drivetrain(_field);
  public Claw _claw = new Claw();
  public Elevator _elevator = new Elevator();
  public LedController _ledController = new LedController();

  /* --- Commands --- */
  // Synchronization commands
  private CommandBase _resetAllCommand = new ResetAllCommand(_claw, _drivetrain, _elevator);

  // Driver controller feedback
  private InstantCommand _xboxRumbleCommand = _controlboard.driverRumbleCommand();
  private InstantCommand _xboxResetRumbleCommand = _controlboard.driverResetRumbleCommand();

  // Operator Commands
  private CommandBase _swapGamePieceCommand = _controlboard.swapDesiredGamePieceCommand();
  private CommandBase _swapIntakeLocationCommand = _controlboard.swapIntakeLocationCommand();

  // Manual operator overrides commands
  private InstantCommand _setHeldGamePieceCommand = _controlboard.setHeldGamePieceCommand();
  private InstantCommand _unsetHeldGamePieceCommand = _controlboard.unsetHeldGamePieceCommand();
  private CommandBase _opOverride =
      new InstantCommand(() -> {
        _ledController.setIntakeMode(
            _controlboard.getDesiredIntakeLocation(), GamePieceType.UNKNOWN_GAME_PIECE
        );
        _controlboard.unsetHeldGamePiece();
      }).ignoringDisable(true);

  // Drivetrain
  private TeleopSwerveCommand _teleopSwerveCommand =
      new TeleopSwerveCommand(_drivetrain, _elevator, _controlboard, DriveModes.FIELD_RELATIVE);
  private InstantCommand _zeroGyroCommand = _drivetrain.zeroGyroCommand();
  private InstantCommand _zeroPoseCommand = _drivetrain.zeroPoseCommand();
  private CommandBase _forceChargingWheelDirection =
      _drivetrain.forceChargingWheelDirectionCommand();

  // LED
  private LedChargingCommand _ledChargingCommand =
      new LedChargingCommand(_ledController, _drivetrain, _controlboard);
  private TeleopLedCommand _teleopLEDCommand = new TeleopLedCommand(_ledController, _controlboard);

  /* Base/complex commands ONLY for testing, not bound to any buttons in a real match */
  private InstantCommand _incrementLedCommand = _ledController.incrementAnimationCommand();

  // Scoring
  private CommandBase _manualScoreCommand = TeleopScoreCommands.manual(
      ScoringLevels.UNKNOWN_SCORING_LEVEL,
      _claw,
      _drivetrain,
      _elevator,
      _ledController,
      _controlboard
  );

  // Go to pose testing
  private GoToPoseCommand _goToPoseCommandPos1 =
      new GoToPoseCommand(_drivetrain, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
  private GoToPoseCommand _goToPoseCommandPos2 =
      new GoToPoseCommand(_drivetrain, new Pose2d(-2.0, -2.0, Rotation2d.fromDegrees(0.0)));
  private CommandBase _dynamicGoToScoringLocation = new DynamicGoToScoringLocationCommand(
      _drivetrain,
      _controlboard,
      false,
      Constants.POST_SCORING_RED_OFFSET,
      ScoringLocations.UNKNOWN_SCORING_LOCATION
  );
  private CommandBase _platformGoToScoringLocation = new DynamicGoToIntakeLocationCommand(
      _drivetrain, _controlboard, new Translation2d(Constants.PLATFORM_PREP_DIST_M, 0.0)
  );
  private InstantCommand _forceSetInitialPose = _drivetrain.forceSetInitalPoseCommand();

  // Intake
  private CommandBase _groundIntake =
      GroundIntakeCommands.teleop(_claw, _elevator, _ledController, _controlboard);
  private CommandBase _manualPlatformIntake =
      PlatformIntakeCommand.manual(_claw, _drivetrain, _elevator, _ledController, _controlboard);
  private CommandBase _manualSingleStationIntake = SingleStationIntakeCommand.manual(
      _claw, _drivetrain, _elevator, _ledController, _controlboard
  );

  // Elevator
  private ElevatorZeroCommand _zeroElevator = new ElevatorZeroCommand(_elevator);
  private ElevatorRaiseToCommand _elevatorDynamic = new ElevatorRaiseToCommand(
      _elevator, _controlboard, ElevatorPositions.ELEVATOR_UNKNOWN_POSITION
  );

  private ElevatorRaiseToCommand _elevatorMin =
      new ElevatorRaiseToCommand(_elevator, _controlboard, ElevatorPositions.ELEVATOR_MIN_TICKS);
  private ElevatorRaiseToCommand _elevatorMax =
      new ElevatorRaiseToCommand(_elevator, _controlboard, ElevatorPositions.ELEVATOR_MAX_TICKS);

  // Elevator Cone
  private ElevatorRaiseToCommand _elevator0Cone =
      new ElevatorRaiseToCommand(_elevator, _controlboard, ElevatorPositions.ELEVATOR_LEVEL_0_CONE);
  private ElevatorRaiseToCommand _elevator1Cone =
      new ElevatorRaiseToCommand(_elevator, _controlboard, ElevatorPositions.ELEVATOR_LEVEL_1_CONE);
  private ElevatorRaiseToCommand _elevator2Cone =
      new ElevatorRaiseToCommand(_elevator, _controlboard, ElevatorPositions.ELEVATOR_LEVEL_2_CONE);
  private ElevatorRaiseToCommand _elevatorConePlatformIntake = new ElevatorRaiseToCommand(
      _elevator, _controlboard, ElevatorPositions.ELEVATOR_PLATFORM_CONE_INTAKE
  );

  // Elevator Cube
  private ElevatorRaiseToCommand _elevator0Cube =
      new ElevatorRaiseToCommand(_elevator, _controlboard, ElevatorPositions.ELEVATOR_LEVEL_0_CUBE);
  private ElevatorRaiseToCommand _elevator1Cube =
      new ElevatorRaiseToCommand(_elevator, _controlboard, ElevatorPositions.ELEVATOR_LEVEL_1_CUBE);
  private ElevatorRaiseToCommand _elevator2Cube =
      new ElevatorRaiseToCommand(_elevator, _controlboard, ElevatorPositions.ELEVATOR_LEVEL_2_CUBE);
  private ElevatorRaiseToCommand _elevatorCubePlatformIntake = new ElevatorRaiseToCommand(
      _elevator, _controlboard, ElevatorPositions.ELEVATOR_PLATFORM_CUBE_INTAKE
  );

  // Claw

  // Outtake
  private CommandBase _shootCone1 =
      new ClawOuttakeCommand(_claw, _controlboard, false, ClawScoreLevels.CONE_LEVEL_1);
  private CommandBase _shootCone2 =
      new ClawOuttakeCommand(_claw, _controlboard, false, ClawScoreLevels.CONE_LEVEL_2);
  private CommandBase _shootCube0 =
      new ClawOuttakeCommand(_claw, _controlboard, false, ClawScoreLevels.CUBE_LEVEL_0);
  private CommandBase _shootCube1 =
      new ClawOuttakeCommand(_claw, _controlboard, false, ClawScoreLevels.CUBE_LEVEL_1);
  private CommandBase _shootCube2 =
      new ClawOuttakeCommand(_claw, _controlboard, false, ClawScoreLevels.CUBE_LEVEL_2);

  // Claw intake command (complex)
  private CommandBase _clawIntake =
      ClawIntakeCommand.ClawIntake(_claw, _controlboard, _ledController);

  /* Shuffleboard */
  // Auto and Path Planner
  private Alliance _alliance = Alliance.Invalid;
  private SendableChooser<Integer> _autoChooser = new SendableChooser<Integer>();
  public AutonomousRoutines _curAutoSelected = AutonomousRoutines.DEFAULT_AUTO;

  private Map<String, Command> _eventMap;
  private AutonomousRoutines[] _autonModes;

  // Verbose mode
  // private double[] _yawPitchRoll = new double[3];
  private double[] _curPoses = new double[4];
  private double[] _visionPoses = new double[4];

  private DataLog _log = DataLogManager.getLog();
  private DoubleArrayLogEntry _poseLog = new DoubleArrayLogEntry(_log, "/Swerve/RobotPose");
  private DoubleArrayLogEntry _visionPoseLog = new DoubleArrayLogEntry(_log, "/Swerve/VisionPose");
  private DoubleArrayLogEntry _gyroLog = new DoubleArrayLogEntry(_log, "/Swerve/Gyro");
  private DoubleArrayLogEntry _swerveSetpoints = new DoubleArrayLogEntry(_log, "/Swerve/Setpoints");
  private DoubleArrayLogEntry _swerveOutputs = new DoubleArrayLogEntry(_log, "/Swerve/RealOutputs");

  private DoubleLogEntry _posXLog = new DoubleLogEntry(_log, "/Drive/X");
  private DoubleLogEntry _posYLog = new DoubleLogEntry(_log, "/Drive/Y");
  private DoubleLogEntry _posThetaLog = new DoubleLogEntry(_log, "/Drive/Theta");

  public RobotContainer() {
    var setupTab = Shuffleboard.getTab("Setup");

    // Auto Chooser
    setupTab.add("Auto Chooser", _autoChooser).withSize(3, 2);
    _autoChooser.setDefaultOption("NO AUTONOMOUS", AutonomousRoutines.DEFAULT_AUTO.ordinal());

    // Default commands
    _drivetrain.setDefaultCommand(_teleopSwerveCommand);
    _ledController.setDefaultCommand(_teleopLEDCommand);

    registerAutonomousCommands();
    registerAutonomousRoutines();

    configureBindings();
    configShuffleboard();

    LoopTimer.markCompletion("\n Robot Initialized: ");
  }

  private void registerAutonomousUtilityCommands() {
    /* --- Print marker commands --- */
    // Use these to log data
    AutonomousAction.registerAutonomousAction("printStart", "Start Auto Sequence");

    /* --- Wait Commands --- */
    // Use these for parallel deadline stop points to stop the drivetrain from moving
    AutonomousAction.registerAutonomousAction("waitScore", 1.75);
    AutonomousAction.registerAutonomousAction("waitScoreCone1", 0.2);
    AutonomousAction.registerAutonomousAction("waitScoreCube1", 0.2);
    AutonomousAction.registerAutonomousAction("waitScoreCube2", 0.2);
    AutonomousAction.registerAutonomousAction("waitIntake", 0.05);
    AutonomousAction.registerAutonomousAction("waitWireScore", 1.0);
    AutonomousAction.registerAutonomousAction("waitCharge", 1.0);

    /* --- LED Commands --- */
    AutonomousAction.registerAutonomousAction("redOrangeLed", _ledController, LedColors.RED_ORANGE);
    AutonomousAction.registerAutonomousAction("yellowLed", _ledController, LedColors.YELLOW);
    AutonomousAction.registerAutonomousAction("pinkLed", _ledController, LedColors.PINK);
    AutonomousAction.registerAutonomousAction("greenLed", _ledController, LedColors.GREEN);
    AutonomousAction.registerAutonomousAction("blueLed", _ledController, LedColors.BLUE);
    AutonomousAction.registerAutonomousAction("purpleLed", _ledController, LedColors.PURPLE);
  }

  private void registerAutonomousScoreCommands() {
    /* --- Score Commands --- */
    // prep
    AutonomousAction.registerAutonomousAction(
        "prepScoreCone",
        AutonomousScoreCommands.prep(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_2_CONE,
            GamePieceType.CONE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "prepScoreCone2",
        AutonomousScoreCommands.prep(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_2_CONE,
            GamePieceType.CONE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "prepScoreCone0",
        AutonomousScoreCommands.prep(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_0_CONE,
            GamePieceType.CONE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "prepScoreCube2",
        AutonomousScoreCommands.prep(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_2_CUBE,
            GamePieceType.CUBE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "prepScoreCube1",
        AutonomousScoreCommands.prep(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_1_CUBE,
            GamePieceType.CUBE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "prepScoreCube0",
        AutonomousScoreCommands.prep(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_0_CUBE,
            GamePieceType.CUBE_GAME_PIECE
        )
    );

    // action
    AutonomousAction.registerAutonomousAction(
        "scoreCube0",
        AutonomousScoreCommands.score(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_0_CUBE,
            ClawScoreLevels.CUBE_LEVEL_0,
            GamePieceType.CUBE_GAME_PIECE,
            5
        )
    );
    AutonomousAction.registerAutonomousAction(
        "scoreCube1",
        AutonomousScoreCommands.score(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_1_CUBE,
            ClawScoreLevels.CUBE_LEVEL_1,
            GamePieceType.CUBE_GAME_PIECE,
            5
        )
    );
    AutonomousAction.registerAutonomousAction(
        "scoreCube2",
        AutonomousScoreCommands.score(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_2_CUBE,
            ClawScoreLevels.CUBE_LEVEL_2,
            GamePieceType.CUBE_GAME_PIECE,
            5
        )
    );
    AutonomousAction.registerAutonomousAction(
        "scoreCone2",
        AutonomousScoreCommands.score(
            _elevator,
            _controlboard,
            _claw,
            _ledController,
            _drivetrain,
            ElevatorPositions.ELEVATOR_LEVEL_2_CONE,
            ClawScoreLevels.CONE_LEVEL_2,
            GamePieceType.CONE_GAME_PIECE,
            5
        )
    );

    // post
    AutonomousAction.registerAutonomousAction(
        "postScore", AutonomousScoreCommands.post(_elevator, _controlboard, _claw, _ledController)
    );

    AutonomousAction.registerAutonomousAction(
        "forceConeShot",
        ClawRollersCommands.on(_claw, ClawDirections.CLAW_ROLLERS_CONE_OUT, 1.0)
            .andThen(Commands.waitSeconds(ScoringLevels.Consts.AUTO_CONE_SHOT_TIMEOUT_S))
    );
    AutonomousAction.registerAutonomousAction(
        "forceCubeShot",
        ClawRollersCommands.on(_claw, ClawDirections.CLAW_ROLLERS_CUBE_OUT, 1.0)
            .andThen(Commands.waitSeconds(ScoringLevels.Consts.AUTO_CUBE_SHOT_TIMEOUT_S))
    );
    // TODO: missing
    // AutonomousAction.registerAutonomousAction(
    //     "wireScore", WireCoverScoreCommands.score(_claw, _controlboard, _elevator, -1.0, 10000)
    // );
    // AutonomousAction.registerAutonomousAction(
    //     "wirePrepScore",
    //     WireCoverPrepScoreCommands.prep(
    //         _claw, _controlboard, _elevator, GamePieceType.CONE_GAME_PIECE
    //     )
    // );
    // AutonomousAction.registerAutonomousAction(
    //     "wirePostScore", WireCoverPostScoreCommands.post(_claw, _controlboard, _elevator)
    // );
  }

  private void registerAutonomousIntakeCommands() {
    /* --- Intake Commands --- */
    // prep
    AutonomousAction.registerAutonomousAction(
        "prepGroundIntake",
        GroundIntakeCommands.autonPrep(
            _claw, _elevator, _controlboard, _ledController, GamePieceType.CUBE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "prepGroundIntakeCube",
        GroundIntakeCommands.autonPrep(
            _claw, _elevator, _controlboard, _ledController, GamePieceType.CUBE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "prepGroundIntakeCone",
        GroundIntakeCommands.autonPrep(
            _claw, _elevator, _controlboard, _ledController, GamePieceType.CONE_GAME_PIECE
        )
    );

    // action
    AutonomousAction.registerAutonomousAction(
        "groundIntake",
        GroundIntakeCommands.auton(
            _claw, _elevator, _ledController, _controlboard, 3, GamePieceType.CUBE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "groundIntakeCube",
        GroundIntakeCommands.auton(
            _claw, _elevator, _ledController, _controlboard, 3, GamePieceType.CUBE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "groundIntakeCone",
        GroundIntakeCommands.auton(
            _claw, _elevator, _ledController, _controlboard, 3, GamePieceType.CONE_GAME_PIECE
        )
    );

    // post
    AutonomousAction.registerAutonomousAction(
        "postGroundIntake",
        GroundIntakeCommands.autonPost(
            _claw, _elevator, _controlboard, _ledController, GamePieceType.CUBE_GAME_PIECE
        )
    );
    AutonomousAction.registerAutonomousAction(
        "postGroundIntakeCone",
        GroundIntakeCommands.autonPost(
            _claw, _elevator, _controlboard, _ledController, GamePieceType.CONE_GAME_PIECE
        )
    );
  }

  private void registerAutonomousCommands() {
    registerAutonomousUtilityCommands();
    registerAutonomousScoreCommands();
    registerAutonomousIntakeCommands();

    _eventMap = AutonomousAction.getEventMap();
  }

  private void registerAutonomousRoutines() {
    if (_autonModes == null) {
      for (Builders builder : Builders.values()) {
        builder.initialize(_drivetrain, _eventMap);
      }

      _autonModes = AutonomousRoutines.values();
      for (AutonomousRoutines routine : _autonModes) {
        if (routine.showInDashboard) {
          routine.build(_drivetrain, _controlboard, _elevator, _claw, _ledController);
          _autoChooser.addOption(routine.name, routine.ordinal());
        }
      }
    }
  }

  public AutonomousRoutines getAutonomousRoutineSelection() {
    var result = _autoChooser.getSelected();
    return _autonModes[result == null ? 0 : result.intValue()];
  }

  public void configFMSData() {
    _alliance = DriverStation.getAlliance();
    _drivetrain.updateAlliance(_alliance);
    _controlboard.updateAlliance(_alliance);
  }

  public void configShuffleboard() {
    /* Commands for pit testing */
    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");

    // Claw
    // tuningTab.add("Claw open", _clawOpenCommand).withPosition(3, 1);
    // tuningTab.add("Claw close", _clawCloseCommand).withPosition(4, 1);

    // Shoot
    tuningTab.add("Shoot cone lv2", _shootCone2).withPosition(3, 2);
    tuningTab.add("Shoot cube lv2", _shootCube2).withPosition(4, 2);

    // // Shoot sequence
    // tuningTab.add("Manual platform intake", _manualPlatformIntakeCommand).withPosition(3, 4);
    // tuningTab.add("Claw intake", _clawIntake).withPosition(4, 4);
    // tuningTab.add("Manualprep to score", _manualPrepToScore).withPosition(5, 4);
    // tuningTab.add("Manual shoot", _manualShoot).withPosition(6, 4);

    // Elevator
    tuningTab.add("Elevator Cone lv0", _elevator0Cone).withPosition(7, 0);
    tuningTab.add("Elevator Cone lv1", _elevator1Cone).withPosition(8, 0);
    tuningTab.add("Elevator Cone lv2", _elevator2Cone).withPosition(9, 0);
    tuningTab.add("Elevator Cube lv0", _elevator0Cube).withPosition(7, 1);
    tuningTab.add("Elevator Cube lv1", _elevator1Cube).withPosition(8, 1);
    tuningTab.add("Elevator Cube lv2", _elevator2Cube).withPosition(9, 1);
    tuningTab.add("Elevator ZERO", _zeroElevator).withPosition(9, 4);

    if (RobotAltModes.isSim) {
      SmartDashboard.putData("Set Held", _setHeldGamePieceCommand);
      SmartDashboard.putData("Unset Held", _unsetHeldGamePieceCommand);
    }
  }

  public void robotInit() {
    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    _ledController.setDisabled(true);
    if (RobotAltModes.isVerboseMode) {
      DriverStation.startDataLog(_log);
    }
  }
  public void autonomousInit() {
    _drivetrain.enableAuto();
    _drivetrain.resetModulesToAbsolute();
    _drivetrain.autonomousDriveMode(true);
    _elevator.onEnable();
  }

  public void teleopInit() {
    _elevator.onEnable();
    _drivetrain.disableAuto();
    _drivetrain.resetModulesToAbsolute();
    _drivetrain.autonomousDriveMode(false);
    _teleopSwerveCommand.teleopInit();
    _resetAllCommand.schedule();
    _ledController.setDisabled(false);
    _ledController.changeAnimation(LedModes.RAINBOW);
  }

  public void simulationInit() {
    _drivetrain.simulationInit();
  }

  public void log() {
    if (RobotAltModes.isVerboseMode) {
      Pose2d curPose = _drivetrain.getPose();
      _curPoses[0] = curPose.getX();
      _curPoses[1] = curPose.getY();
      _curPoses[2] = curPose.getRotation().getDegrees();
      _poseLog.append(_curPoses);

      Pose2d visionPose = _drivetrain.getVisionPose();
      _visionPoses[0] = visionPose.getX();
      _visionPoses[1] = visionPose.getY();
      _visionPoses[2] = visionPose.getRotation().getDegrees();
      _visionPoseLog.append(_visionPoses);

      // might be problematic, look into
      // _drivetrain.getYawPitchRoll(_yawPitchRoll);
      // _gyroLog.append(_yawPitchRoll);

      // Module Config:
      // Swerve chassis FL, BL, BR, FR
      // Practice bot FL, FR, BL, BR
      _swerveOutputs.append(_drivetrain.swerveMeasuredIO());
      _swerveSetpoints.append(_drivetrain.swerveSetpointsIO());

      _posXLog.append(curPose.getX());
      _posYLog.append(curPose.getY());
      _posThetaLog.append(curPose.getRotation().getDegrees());
    }
  }

  public void periodic() {
    _controlboard.updateOperatorSelections();
    _controlboard.updateShuffleboard();
  }

  public void disabledPeriodic() {
    AutonomousRoutines prev = _curAutoSelected;
    _curAutoSelected = getAutonomousRoutineSelection();

    Alliance prevAlliance = _alliance;
    _alliance = DriverStation.getAlliance();

    if (_curAutoSelected == AutonomousRoutines.DEFAULT_AUTO) {
      // No auto selected, turn entire strip red
      _ledController.setSolidColor(LedColors.RED_ORANGE, LedSections.ALL);
    } else {
      // For initial pose aligning
      if (prev != _curAutoSelected || prevAlliance != _alliance) {
        _drivetrain.updateAlliance(_alliance);
        _controlboard.updateAlliance(_alliance);

        _ledController.setSolidColor(
            0,
            0,
            0,
            // _curAutoSelected.color.r, _curAutoSelected.color.g, _curAutoSelected.color.b,
            LedConfs.LED_WHITE_LEVEL,
            LedSections.CANDLE
        );
      }

      _ledController.disabledPeriodic(
          _drivetrain.getVisionPose(),
          _curAutoSelected.getInitialPose(_alliance),
          _alliance == Alliance.Blue
      );
    }
  }

  public void onEnable() {}

  public void onDisable() {
    resetAll();
    _elevator.onDisable();
    _controlboard.driverResetRumble();
    _ledController.turnOffLeds(LedSections.ALL);
  }

  public void resetAll() {
    // Upon disabling, open all control loops
    _resetAllCommand.schedule();
  }

  public void setLedColor(LedColors ledColor, LedSections ledSection) {
    // does this schedule?
    _ledController.setSolidColorCommand(ledColor, ledSection).schedule();
  }

  public void setLedColor(LedColors ledColor) {
    // does this schedule?
    _ledController.setSolidColorCommand(ledColor).schedule();
  }

  public void runAutonomousCommand(AutoCommand command) {}

  private void configureBindings() {
    // DRIVER
    _controlboard._xboxDrive.x().onTrue(_zeroGyroCommand);
    _controlboard._xboxDrive.start().onTrue(_zeroPoseCommand);

    // intake
    _controlboard._xboxDrive.rightTrigger().whileTrue(_groundIntake);
    _controlboard._xboxDrive.y().toggleOnTrue(_manualPlatformIntake);
    _controlboard._xboxDrive.rightBumper().whileTrue(_manualSingleStationIntake);

    // score
    _controlboard._xboxDrive.leftTrigger().onTrue(_manualScoreCommand);

    // OPERATOR
    _controlboard._ps4Op.L1().onTrue(_swapGamePieceCommand);
    _controlboard._ps4Op.R1().onTrue(_swapIntakeLocationCommand);
    _controlboard._ps4Op.R2().onTrue(_opOverride);

    // ------------------------------------------------------------------------

    /*
    BUTTON KEY
    expected button = actual ps4 button
    cross = circle
    square = cross
    circle = square
    triangle = triangle
    L1 = L1
    R1 = R1
    L2 = share
    R2 = options
    share = can’t use bc it only uses boolean events
    options = right joystick
    no L3
    no R3
    no PS
    no touchpad (edited)
    */

    // PIT
    // _controlboard._xboxDrive.leftStick().onTrue();
    // _controlboard._xboxDrive.rightStick().onTrue();
    // _controlboard._xboxDrive.leftTrigger().onTrue();
    // _controlboard._xboxDrive.rightTrigger().onTrue();
    // _controlboard._xboxDrive.a().onTrue();
    // _controlboard._xboxDrive.b().onTrue();
    // _controlboard._xboxDrive.x().onTrue();
    // _controlboard._xboxDrive.y().onTrue();
    // _controlboard._xboxDrive.povUp().onTrue();
    // _controlboard._xboxDrive.povDown().onTrue();
    // _controlboard._xboxDrive.povLeft().onTrue();
    // _controlboard._xboxDrive.povRight().onTrue();
    // _controlboard._xboxDrive.back().onTrue();
    // _controlboard._xboxDrive.start().onTrue();

    // ------------------------------------------------------------------------

    // // elevator bring up 09/13/2023 #439
    // _controlboard._xboxDrive.a().onTrue(newElevatorRaiseToCommand(
    //     _elevator, _controlboard, ElevatorPositions.ELEVATOR_UNKNOWN_POSITION
    // )); // 0
    // _controlboard._xboxDrive.b().onTrue(
    //     new ElevatorRaiseToCommand(_elevator, _controlboard,
    //     ElevatorPositions.ELEVATOR_STOW_TICKS)
    // ); // 1000
    // _controlboard._xboxDrive.leftBumper().onTrue(_elevator1Cone); // 46400
    // _controlboard._xboxDrive.rightBumper().onTrue(_elevator2Cone); // 66550
  }
}
