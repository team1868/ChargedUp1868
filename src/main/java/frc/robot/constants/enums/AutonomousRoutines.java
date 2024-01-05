package frc.robot.constants.enums;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutonomousScoreCommands;
import frc.robot.commands.LedCommands;
import frc.robot.commands.base.ChargingStationBalanceCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;
import frc.robot.utils.PIDConstants;
import java.util.Map;
import java.util.Vector;

public enum AutonomousRoutines {
  DEFAULT_AUTO(false, "DEFAULT", LedColors.BLOOD_RED, Commands.print("DEFAULT AUTO SAYS HI")),

  // Wire cover shot autos
  S1_I1_W_AUTO(
      false,
      "S1 to I1 to W (CC)",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_1_TO_INTAKE_CONE_1, PathLegs.INTAKE_1_TO_WIRESHOT}
  ),
  S1_I1_W_I2_AUTO(
      false,
      "S1 to I1 to W to I2 (CCC)",
      LedColors.OFF,
      new PathLegs[] {
          PathLegs.SCORE_1_TO_INTAKE_CONE_1,
          PathLegs.INTAKE_1_TO_WIRESHOT,
          PathLegs.WIRESHOT_TO_INTAKE_2}
  ),
  S1_I1_W_I2_C_AUTO(
      false,
      "S1 to I1 to W to I2 to C (CCC)",
      LedColors.OFF,
      new PathLegs[] {
          PathLegs.SCORE_1_TO_INTAKE_CONE_1,
          PathLegs.INTAKE_1_TO_WIRESHOT,
          PathLegs.WIRESHOT_TO_INTAKE_2,
          PathLegs.INTAKE_2_TO_CHARGE_2},
      Builders.AUTO_BUILDER,
      true,
      false
  ),
  S1_I1_W_I2_S2_AUTO(
      false,
      "S1 to I1 to W to I2 to S2 (CCC)",
      LedColors.OFF,
      new PathLegs[] {
          PathLegs.SCORE_1_TO_INTAKE_CONE_1,
          PathLegs.INTAKE_1_TO_WIRESHOT,
          PathLegs.WIRESHOT_TO_INTAKE_2,
          PathLegs.INTAKE_2_TO_SCORE_2}
  ),

  // 0 piece + charge station balance
  C_AUTO(
      false,
      "C ONLY",
      LedColors.SANDY_BROWN,
      new PathLegs[] {PathLegs.SCORE_TO_CHARGE},
      Builders.DEFAULT_BUILDER,
      true,
      false
  ),

  // 1 piece + leave
  S9_L_AUTO(false, "S9 to L", LedColors.DARK_ORANGE, new PathLegs[] {PathLegs.SCORE_9_TO_LOADING}),

  // 1 piece + charge
  S4_M_C_AUTO(
      true,
      "S4 to M to C (C)",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_4_TO_MOBILITY, PathLegs.MOBILITY_TO_CHARGE_4},
      Builders.DEFAULT_BUILDER,
      true,
      false
  ),
  S6_M_C_AUTO(
      true,
      "S6 to M to C (C)",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_6_TO_MOBILITY, PathLegs.MOBILITY_TO_CHARGE_6},
      Builders.DEFAULT_BUILDER,
      true,
      false
  ),

  // 1 piece + charge station balance
  S1_C_AUTO(
      false,
      "S1 to C",
      LedColors.CHARTREUSE,
      new PathLegs[] {PathLegs.SCORE_1_TO_CHARGE_2, PathLegs.SCORE_TO_CHARGE}
  ),
  S2_C_AUTO(
      false,
      "S2 (CUBE) to C",
      LedColors.PINK_LAVENDAR,
      new PathLegs[] {PathLegs.SCORE_2_TO_CHARGE_2, PathLegs.SCORE_TO_CHARGE}
  ),
  S3_C_AUTO(
      false,
      "S3 to C",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_3_TO_CHARGE_2, PathLegs.SCORE_TO_CHARGE}
  ),
  S4_C_AUTO(
      false,
      "S4 to C",
      LedColors.CADET_GREY,
      new PathLegs[] {PathLegs.SCORE_4_TO_CHARGE_2, PathLegs.SCORE_TO_CHARGE}
  ),
  S6_C_AUTO(
      false,
      "S6 to C",
      LedColors.BLOOD_RED,
      new PathLegs[] {PathLegs.SCORE_6_TO_CHARGE_2, PathLegs.SCORE_TO_CHARGE}
  ),
  S7_C_AUTO(
      false,
      "S7 to C",
      LedColors.ELECTRIC_VIOLET,
      new PathLegs[] {PathLegs.SCORE_7_TO_CHARGE_2, PathLegs.SCORE_TO_CHARGE}
  ),
  S8_C_AUTO(
      false,
      "S8 (CUBE) to C",
      LedColors.INCHWORM,
      new PathLegs[] {PathLegs.SCORE_8_TO_CHARGE_2, PathLegs.SCORE_TO_CHARGE}
  ),
  S9_C_AUTO(
      false,
      "S9 to C",
      LedColors.YELLOW,
      new PathLegs[] {PathLegs.SCORE_9_TO_CHARGE_2, PathLegs.SCORE_TO_CHARGE}
  ),

  // 1 piece + leave + charge station balance
  S1_L_C_AUTO(
      false,
      "S1 LEAVE to C",
      LedColors.RAZZLE_DAZZLE_ROSE,
      new PathLegs[] {PathLegs.SCORE_1_TO_CHARGE_2, PathLegs.INTAKE_TO_CHARGE},
      Builders.CHARGING_BUILDER,
      true,
      true
  ),
  S9_L_C_AUTO(
      false,
      "S9 LEAVE to C",
      LedColors.SAFFRON,
      new PathLegs[] {PathLegs.SCORE_9_TO_CHARGE_1, PathLegs.INTAKE_TO_CHARGE},
      Builders.CHARGING_BUILDER,
      true,
      true
  ),

  // 1.5 piece
  S1_I1_AUTO(
      false, "S1 to I1 NO C", LedColors.OFF, new PathLegs[] {PathLegs.SCORE_1_TO_INTAKE_CONE_1}
  ),
  S4_I2_AUTO(
      false,
      "S4 to I2 NO C (CC)",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_4_TO_INTAKE_2},
      Builders.CHARGING_BUILDER
  ),
  S6_I3_AUTO(
      false,
      "S6 to I3 NO C (CC)",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_6_TO_INTAKE_3, PathLegs.MOBILITY_INTAKE_3_TO_CHARGE},
      Builders.CHARGING_BUILDER
  ),

  // 1.5 piece + charge station balance
  S1_I1_C_AUTO(
      false,
      "S1 to I1 to C",
      LedColors.AZURE,
      new PathLegs[] {
          PathLegs.SCORE_1_TO_INTAKE_1, PathLegs.INTAKE_1_TO_CHARGE_2, PathLegs.INTAKE_TO_CHARGE},
      Builders.CHARGING_BUILDER,
      true,
      true
  ),
  S4_I2_C_AUTO(
      false,
      "S4 to I2 to C",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_4_TO_INTAKE_2, PathLegs.MOBILITY_INTAKE_2_TO_CHARGE},
      Builders.CHARGING_BUILDER,
      true,
      true
  ),
  S6_I3_C_AUTO(
      false,
      "S6 to I3 to C",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_6_TO_INTAKE_3, PathLegs.MOBILITY_INTAKE_3_TO_CHARGE},
      Builders.CHARGING_BUILDER,
      true,
      true
  ),
  S9_I4_C_AUTO(
      false,
      "S9 to I4 to C",
      LedColors.AQUA,
      new PathLegs[] {
          PathLegs.SCORE_9_TO_INTAKE_4, PathLegs.INTAKE_4_TO_CHARGE_2, PathLegs.INTAKE_TO_CHARGE},
      Builders.CHARGING_BUILDER,
      true,
      true
  ),

  // 1.5 piece + yeet + charge station balance
  S4_I2_C_Y_AUTO(
      true,
      "S4 to I2 to C Y (CC)",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_4_TO_INTAKE_2, PathLegs.MOBILITY_INTAKE_2_TO_CHARGE},
      Builders.CHARGING_BUILDER,
      true,
      true
  ),
  S6_I3_C_Y_AUTO(
      true,
      "S6 to I3 to C Y (CC)",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_6_TO_INTAKE_3, PathLegs.MOBILITY_INTAKE_3_TO_CHARGE},
      Builders.CHARGING_BUILDER,
      true,
      true
  ),

  // 2 piece
  S1_I1_S2_AUTO(
      false,
      "S1 to I1 to S2",
      LedColors.BLUE,
      new PathLegs[] {PathLegs.SCORE_1_TO_INTAKE_1, PathLegs.INTAKE_1_TO_SCORE_2}
  ),
  S3_I1_S2_AUTO(
      false,
      "S3 to I1 to S2",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_3_TO_INTAKE_1, PathLegs.INTAKE_1_TO_SCORE_2}
  ),
  S4_I1_S2_AUTO(
      false,
      "S4 to I1 to S2",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_4_TO_INTAKE_1, PathLegs.INTAKE_1_TO_SCORE_2}
  ),
  S4_I2_S2_AUTO(
      false,
      "S4 to I2 to S2",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_4_TO_INTAKE_2, PathLegs.INTAKE_2_TO_SCORE_2}
  ),
  S6_I3_S8_AUTO(
      false,
      "S6 to I3 to S8",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_6_TO_INTAKE_3, PathLegs.INTAKE_3_TO_SCORE_8}
  ),
  S6_I4_S8_AUTO(
      false,
      "S6 to I4 to S8",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_6_TO_INTAKE_4, PathLegs.INTAKE_4_TO_SCORE_8}
  ),
  S7_I4_S8_AUTO(
      false,
      "S7 to I4 to S8",
      LedColors.OFF,
      new PathLegs[] {PathLegs.SCORE_7_TO_INTAKE_4, PathLegs.INTAKE_4_TO_SCORE_8}
  ),
  S9_I4_S8_AUTO(
      false,
      "S9 to I4 to S8 (CQ)",
      LedColors.DARK_GREEN,
      new PathLegs[] {PathLegs.SCORE_9_TO_INTAKE_4, PathLegs.INTAKE_4_TO_SCORE_8}
  ),

  // 2 piece + charge station balance
  S1_I1_S2_C_AUTO(
      true,
      "S1 to I1 to S2 to C",
      LedColors.MEDIUM_SPRING_GREEN,
      new PathLegs[] {
          PathLegs.SCORE_1_TO_INTAKE_1, PathLegs.INTAKE_1_TO_SCORE_2, PathLegs.SCORE_2_TO_CHARGE_2},
      Builders.DEFAULT_BUILDER,
      true,
      false
  ),
  S9_I4_S8_C_AUTO(
      false,
      "S9 to I4 to S8 to C (CQ)",
      LedColors.FUSCHIA,
      new PathLegs[] {
          PathLegs.SCORE_9_TO_INTAKE_4, PathLegs.INTAKE_4_TO_SCORE_8, PathLegs.SCORE_8_TO_CHARGE_2},
      Builders.DEFAULT_BUILDER,
      true,
      false
  ),

  // 2.5 piece
  S1_I1_S2_I2_AUTO(
      false,
      "S1 to I1 to S2 to I2",
      LedColors.LESS_LIME,
      new PathLegs[] {
          PathLegs.SCORE_1_TO_INTAKE_1, PathLegs.INTAKE_1_TO_SCORE_2, PathLegs.SCORE_2_TO_INTAKE_2}
  ),

  // 2.5 piece + charge station balance
  // CHEZY DEV
  S1_I1_S2_I2_C_AUTO(
      true,
      "S1 to I1 to S2 to I2 to C",
      LedColors.MAGENTA,
      new PathLegs[] {
          PathLegs.SCORE_1_TO_INTAKE_1,
          PathLegs.INTAKE_1_TO_SCORE_2,
          PathLegs.SCORE_2_TO_INTAKE_2,
          PathLegs.INTAKE_2_TO_CHARGE_2},
      Builders.DEFAULT_BUILDER,
      true,
      false
  ),
  S9_I4_S8_I3_C_AUTO(
      false,
      "S9 to I4 to S8 to I3 to C (CQQ)",
      LedColors.WHITE,
      new PathLegs[] {
          PathLegs.SCORE_9_TO_INTAKE_4,
          PathLegs.INTAKE_4_TO_SCORE_8,
          PathLegs.SCORE_8_TO_INTAKE_3,
          PathLegs.INTAKE_3_TO_CHARGE_2},
      Builders.AUTO_BUILDER,
      true,
      false
  ),
  S9_I4_S8_I3_CONE_C_AUTO(
      true,
      "S9 to I4 to S8 to I3 CONE to C (CQC)",
      LedColors.OFF,
      new PathLegs[] {
          PathLegs.SCORE_9_TO_INTAKE_4,
          PathLegs.INTAKE_4_TO_SCORE_8,
          PathLegs.SCORE_8_TO_INTAKE_3_CONE,
          PathLegs.INTAKE_3_CONE_TO_CHARGE_2},
      Builders.AUTO_BUILDER,
      true,
      false
  ),

  // 3 piece
  // CHEZY DEV
  S1_I1_S2_I2_S1_AUTO(
      true,
      "S1 to I1 to S2 to I2 to S1",
      LedColors.BLACK,
      new PathLegs[] {
          PathLegs.SCORE_1_TO_INTAKE_1,
          PathLegs.INTAKE_1_TO_SCORE_2,
          PathLegs.SCORE_2_TO_INTAKE_2,
          PathLegs.INTAKE_2_TO_SCORE_1}
  ),
  S1_I1_S2_I2_S2_AUTO(
      false,
      "S1 to I1 to S2 to I2 to S2",
      LedColors.BLACK,
      new PathLegs[] {
          PathLegs.SCORE_1_TO_INTAKE_1,
          PathLegs.INTAKE_1_TO_SCORE_2,
          PathLegs.SCORE_2_TO_INTAKE_2,
          PathLegs.INTAKE_2_TO_SCORE_2}
  ),
  S9_I4_S8_I3_S8_AUTO(
      false,
      "S9 to I4 to S8 to I3 to S8 (CQQ)",
      LedColors.LIME,
      new PathLegs[] {
          PathLegs.SCORE_9_TO_INTAKE_4,
          PathLegs.INTAKE_4_TO_SCORE_8,
          PathLegs.SCORE_8_TO_INTAKE_3_CONE}
  ),
  S9_I4_S8_I3_S9_CQC_AUTO(
      "S9 to I4 to S8 to I3 CONE to S9 (CQC)",
      LedColors.OFF,
      new PathLegs[] {
          PathLegs.SCORE_9_TO_INTAKE_4,
          PathLegs.INTAKE_4_TO_SCORE_8,
          PathLegs.SCORE_8_TO_INTAKE_3_CONE,
          PathLegs.INTAKE_3_TO_SCORE_9_CONE}
  ),
  S9_SCORE_3_LINK_AUTO(
      false,
      "S9 Score 3 Link",
      LedColors.SEA_GREEN,
      new PathLegs[] {
          PathLegs.SCORE_9_TO_INTAKE_4,
          PathLegs.INTAKE_4_TO_SCORE_8,
          PathLegs.SCORE_8_TO_INTAKE_3_CONE,
          PathLegs.INTAKE_3_TO_SCORE_7}
  ),
  /* hybrid auto for testing */
  S1_I1_VIS_S2_AUTO(
      true,
      "S1 to I1 to S2 Hybrid Auto",
      LedColors.BLOOD_RED,
      new PathLegs[] {PathLegs.SCORE_1_TO_INTAKE_1},
      new PathLegs[] {PathLegs.INTAKE_1_TO_SCORE_2},
      Builders.DEFAULT_BUILDER
  );

  public final boolean showInDashboard;
  public final String name;
  public final LedColors color;
  public final boolean buildable;
  public final Builders builder;
  public final Builders builder2;
  public final boolean balanceAtEnd;
  public final boolean yeetCone;

  private PathLegs[] _paths;
  private PathLegs[] _paths2;
  private PathPlannerState initialState;
  private Pose2d initialRedPose, initialBluePose;
  public CommandBase command;
  public CommandBase builtCommand;
  public CommandBase builtCommandVision;

  AutonomousRoutines(String shuffleboardName, LedColors color, CommandBase command) {
    this(true, shuffleboardName, color, command);
  }
  AutonomousRoutines(boolean show, String shuffleboardName, LedColors color, CommandBase command) {
    this.showInDashboard = show;
    this.name = shuffleboardName;
    this.color = color;
    this.command = command;

    this.buildable = false;
    this.builder = null;
    this.builder2 = null;
    this.balanceAtEnd = false;
    this.yeetCone = false;
  }

  AutonomousRoutines(String shuffleboardName, PathLegs[] paths) {
    this(true, shuffleboardName, paths);
  }

  AutonomousRoutines(boolean show, String shuffleboardName, PathLegs[] paths) {
    this(show, shuffleboardName, LedColors.OFF, paths);
  }

  AutonomousRoutines(String shuffleboardName, LedColors color, PathLegs[] paths) {
    this(true, shuffleboardName, color, paths);
  }

  AutonomousRoutines(boolean show, String shuffleboardName, LedColors color, PathLegs[] paths) {
    this(show, shuffleboardName, color, paths, Builders.DEFAULT_BUILDER);
  }

  AutonomousRoutines(String shuffleboardName, LedColors color, PathLegs[] paths, Builders builder) {
    this(true, shuffleboardName, color, paths, builder);
  }

  AutonomousRoutines(
      boolean show, String shuffleboardName, LedColors color, PathLegs[] paths, Builders builder
  ) {
    this(show, shuffleboardName, color, paths, builder, false, false);
  }

  AutonomousRoutines(
      boolean show,
      String shuffleboardName,
      LedColors color,
      PathLegs[] paths,
      Builders builder,
      boolean balance,
      boolean yeet
  ) {
    this.showInDashboard = show;
    this.name = shuffleboardName;
    this.color = color;

    this.buildable = builder != null && paths.length > 0;
    this.builder = builder;
    this.builder2 = null;
    this._paths = paths;
    this._paths2 = null;

    this.balanceAtEnd = balance;
    this.yeetCone = yeet;
  }

  AutonomousRoutines(
      boolean show,
      String shuffleboardName,
      LedColors color,
      PathLegs[] preVisPaths,
      PathLegs[] postVisPaths,
      Builders builder
  ) {
    this.showInDashboard = show;
    this.name = shuffleboardName;
    this.color = color;

    this.buildable = builder != null && (preVisPaths.length + postVisPaths.length) > 0;
    this.builder = null;
    this.builder2 = builder;
    this._paths = preVisPaths;
    this._paths2 = postVisPaths;

    this.balanceAtEnd = false;
    this.yeetCone = false;
  }

  public void build(
      Drivetrain drivetrain,
      Controlboard controlboard,
      Elevator elevator,
      Claw claw,
      LedController ledController
  ) {
    if (_paths2 != null) {
      build2(drivetrain, controlboard, elevator, claw, ledController);
    } else if (buildable) {
      Vector<PathPlannerTrajectory> pathSet = new Vector<PathPlannerTrajectory>();
      for (PathLegs p : _paths) {
        if (p.currentlyExists) {
          pathSet.add(PathPlanner.loadPath(p.name, p.constraint.pathPlanner));
        } else {
          System.out.println("Auto routine with currently undefined path " + p.name);
          System.exit(1);
        }
      }
      initialState = pathSet.get(0).getInitialState();
      Pose2d initialPose = initialState.poseMeters;
      initialBluePose =
          new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180.0));
      initialRedPose = new Pose2d(
          Constants.CField.dims.x_M - initialPose.getX(),
          initialPose.getY(),
          Rotation2d.fromDegrees(0.0)
      );

      builtCommand = builder.construct(pathSet)
                         .finallyDo((boolean interrupted) -> drivetrain.autoZeroGyro())
                         .beforeStarting(() -> {
                           controlboard.setHeldGamePiece(GamePieceType.CONE_GAME_PIECE);
                           drivetrain.setAutoPrepScore(false);
                         });

      // The order of these matters, a lot

      if (balanceAtEnd && yeetCone) {
        command = Commands.sequence(
            builtCommand,
            new ChargingStationBalanceCommand(drivetrain, controlboard),
            AutonomousScoreCommands.yeet(elevator, controlboard, claw, ClawScoreLevels.CONE_LEVEL_2)
                .asProxy()
        );
      } else if (balanceAtEnd) {
        command = Commands.sequence(
            builtCommand, new ChargingStationBalanceCommand(drivetrain, controlboard)
        );
      } else if (yeetCone) {
        command = Commands.sequence(
            builtCommand,
            AutonomousScoreCommands.yeet(elevator, controlboard, claw, ClawScoreLevels.CONE_LEVEL_2)
                .asProxy()
        );
      } else {
        command = builtCommand;
      }
    }
  }

  public void build2(
      Drivetrain drivetrain,
      Controlboard controlboard,
      Elevator elevator,
      Claw claw,
      LedController ledController
  ) {
    if (buildable) {
      Vector<PathPlannerTrajectory> pathSet = new Vector<PathPlannerTrajectory>();
      for (PathLegs p : _paths) {
        if (p.currentlyExists) {
          pathSet.add(PathPlanner.loadPath(p.name, p.constraint.pathPlanner));
        } else {
          System.out.println("Auto routine with currently undefined path " + p.name);
          System.exit(1);
        }
      }

      initialState = pathSet.get(0).getInitialState();
      Pose2d initialPose = initialState.poseMeters;
      initialBluePose =
          new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180.0));
      initialRedPose = new Pose2d(
          Constants.CField.dims.x_M - initialPose.getX(),
          initialPose.getY(),
          Rotation2d.fromDegrees(0.0)
      );

      Vector<PathPlannerTrajectory> pathSet2 = new Vector<PathPlannerTrajectory>();
      for (PathLegs p : _paths2) {
        if (p.currentlyExists) {
          pathSet2.add(PathPlanner.loadPath(p.name, p.constraint.pathPlanner));
        } else {
          System.out.println("Auto routine with currently undefined path " + p.name);
          System.exit(1);
        }
      }

      builtCommand = builder.construct(pathSet).beforeStarting(() -> {
        controlboard.setHeldGamePiece(GamePieceType.CONE_GAME_PIECE);
        drivetrain.setAutoPrepScore(false);
      });

      builtCommandVision =
          builder2.construct2(pathSet2)
              .finallyDo((boolean interrupted) -> drivetrain.autoZeroGyro())
              .beforeStarting(
                  () -> drivetrain.setPose(pathSet2.get(0).getInitialState().poseMeters)
              );

      command = Commands.sequence(
          builtCommand,
          ledController.setColorCommand(LedColors.AQUA, LedSections.ALL),
          builtCommandVision
      );
    }
  }

  public void configureCommand(CommandBase command) {
    this.command = command;
  }

  public Pose2d getInitialPose(Alliance alliance) {
    return alliance == Alliance.Blue ? initialBluePose : initialRedPose;
  }

  public enum Builders {
    DEFAULT_BUILDER(Constants.CRobot.drive.control.xy, Constants.CRobot.drive.control.theta),
    AUTO_BUILDER(Constants.CRobot.drive.control.xy, Constants.CRobot.drive.control.autonTheta),
    CHARGING_BUILDER(
        Constants.CRobot.drive.control.xy, Constants.CRobot.drive.control.chargerAutonTheta
    );

    private SwerveAutoBuilder builder;
    private SwerveAutoBuilder builder2;
    private final PIDConstants xy;
    private final PIDConstants theta;

    Builders(PIDConstants xy, PIDConstants theta) {
      this.xy = xy;
      this.theta = theta;
    }

    public void initialize(Drivetrain drivetrain, Map<String, Command> eventMap) {
      builder = new SwerveAutoBuilder(
          ()
              -> { return drivetrain.getPose(); },
          (Pose2d initPose)
              -> { drivetrain.setPose(initPose, initPose.getRotation()); },
          drivetrain.getSwerveKinematics(),
          xy.getPathPlannerTranslation(),
          theta.getPathPlannerTheta(),
          (SwerveModuleState[] states)
              -> { drivetrain.setModuleStates(states); },
          eventMap,
          true,
          drivetrain
      );
      builder2 = new SwerveAutoBuilder(
          ()
              -> { return drivetrain.getPose(); },
          (Pose2d initPose)
              -> { drivetrain.setPose(initPose); },
          drivetrain.getSwerveKinematics(),
          xy.getPathPlannerTranslation(),
          xy.getPathPlannerTheta(),
          (SwerveModuleState[] states)
              -> { drivetrain.setModuleStates(states); },
          eventMap,
          true,
          drivetrain
      );
    }

    public CommandBase construct(Vector<PathPlannerTrajectory> trajectories) {
      return builder.fullAuto(trajectories);
    }

    public CommandBase construct2(Vector<PathPlannerTrajectory> trajectories) {
      return builder2.fullAuto(trajectories);
    }
  }

  public enum PPPConstraints {
    AUTO_DEFAULT(3.4, 1.8),
    CHARGE(2.0, 2.0),
    CHARGE_TRAVERSAL(1.4, 1.6);

    public final com.pathplanner.lib.PathConstraints pathPlanner;

    PPPConstraints(double maxVelo_mps, double maxAccel_mps2) {
      pathPlanner = new com.pathplanner.lib.PathConstraints(maxVelo_mps, maxAccel_mps2);
    }
  }

  public enum PathLegs {
    INTAKE_1_TO_WIRESHOT("INTAKE_2_TO_CHARGE_2", PPPConstraints.AUTO_DEFAULT),
    INTAKE_1_TO_SCORE_2("INTAKE_1_TO_SCORE_2", PPPConstraints.AUTO_DEFAULT),
    INTAKE_2_TO_CHARGE_2("INTAKE_2_TO_CHARGE_2", PPPConstraints.CHARGE),
    INTAKE_2_TO_SCORE_1("INTAKE_2_TO_SCORE_1", PPPConstraints.AUTO_DEFAULT),
    INTAKE_2_TO_SCORE_2("INTAKE_2_TO_SCORE_2", PPPConstraints.AUTO_DEFAULT),
    INTAKE_3_CONE_TO_CHARGE_2("INTAKE_3_CONE_TO_CHARGE_2", PPPConstraints.AUTO_DEFAULT),
    INTAKE_3_TO_SCORE_8("INTAKE_3_TO_SCORE_8", PPPConstraints.AUTO_DEFAULT),
    INTAKE_3_TO_SCORE_9_CONE("INTAKE_3_TO_SCORE_9_CONE", PPPConstraints.AUTO_DEFAULT),
    INTAKE_4_TO_SCORE_8("INTAKE_4_TO_SCORE_8", PPPConstraints.AUTO_DEFAULT),
    MOBILITY_INTAKE_2_TO_CHARGE("MOBILITY_INTAKE_2_TO_CHARGE", PPPConstraints.CHARGE),
    MOBILITY_INTAKE_3_TO_CHARGE("MOBILITY_INTAKE_3_TO_CHARGE", PPPConstraints.CHARGE),
    MOBILITY_TO_CHARGE_4("MOBILITY_TO_CHARGE_4", PPPConstraints.AUTO_DEFAULT),
    MOBILITY_TO_CHARGE_6("MOBILITY_TO_CHARGE_6", PPPConstraints.CHARGE),
    SCORE_1_TO_INTAKE_1("SCORE_1_TO_INTAKE_1", PPPConstraints.AUTO_DEFAULT),
    SCORE_1_TO_INTAKE_CONE_1("SCORE_1_TO_INTAKE_CONE_1", PPPConstraints.AUTO_DEFAULT),
    SCORE_4_TO_INTAKE_2("SCORE_4_TO_INTAKE_2", PPPConstraints.CHARGE_TRAVERSAL),
    SCORE_4_TO_MOBILITY("SCORE_4_TO_MOBILITY", PPPConstraints.CHARGE_TRAVERSAL),
    SCORE_6_TO_INTAKE_3("SCORE_6_TO_INTAKE_3", PPPConstraints.CHARGE_TRAVERSAL),
    SCORE_6_TO_MOBILITY("SCORE_6_TO_MOBILITY", PPPConstraints.CHARGE_TRAVERSAL),
    SCORE_8_TO_INTAKE_3_CONE("SCORE_8_TO_INTAKE_3_CONE", PPPConstraints.AUTO_DEFAULT),
    SCORE_8_TO_INTAKE_3("SCORE_8_TO_INTAKE_3", PPPConstraints.AUTO_DEFAULT),
    SCORE_9_TO_INTAKE_4("SCORE_9_TO_INTAKE_4", PPPConstraints.AUTO_DEFAULT),
    WIRESHOT_TO_INTAKE_2("WIRESHOT_TO_INTAKE_2", PPPConstraints.AUTO_DEFAULT),
    SCORE_TO_CHARGE(false, "SCORE_TO_CHARGE", PPPConstraints.CHARGE),
    INTAKE_TO_CHARGE(false, "INTAKE_TO_CHARGE", PPPConstraints.CHARGE),
    INTAKE_TOP_TO_CHARGE(false, "INTAKE_TOP_TO_CHARGE", PPPConstraints.CHARGE),
    INTAKE_BOTTOM_TO_CHARGE(false, "INTAKE_BOTTOM_TO_CHARGE", PPPConstraints.CHARGE),
    INTAKE_3_TO_CHARGE_2(false, "INTAKE_3_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_2_TO_CHARGE_2("SCORE_2_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_8_TO_CHARGE_2(false, "SCORE_8_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_TO_CHARGE_ALIGN_I3(false, "SCORE_TO_CHARGE_ALIGN_I3", PPPConstraints.CHARGE),
    SCORE_6_MOBILITY_TO_CHARGE(false, "SCORE_6_MOBILITY_TO_CHARGE", PPPConstraints.CHARGE),
    SCORE_9_TO_LOADING(false, "SCORE_9_TO_LOADING", PPPConstraints.CHARGE),
    SCORE_1_TO_CHARGE_2(false, "SCORE_1_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_3_TO_CHARGE_2(false, "SCORE_3_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_4_TO_CHARGE_2(false, "SCORE_4_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_6_TO_CHARGE_2(false, "SCORE_6_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_7_TO_CHARGE_2(false, "SCORE_7_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_9_TO_CHARGE_2(false, "SCORE_9_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_9_TO_CHARGE_1(false, "SCORE_9_TO_CHARGE_1", PPPConstraints.CHARGE),
    INTAKE_1_TO_CHARGE_2(false, "INTAKE_1_TO_CHARGE_2", PPPConstraints.CHARGE),
    INTAKE_4_TO_CHARGE_2(false, "INTAKE_4_TO_CHARGE_2", PPPConstraints.CHARGE),
    SCORE_3_TO_INTAKE_1(false, "SCORE_3_TO_INTAKE_1", PPPConstraints.AUTO_DEFAULT),
    SCORE_4_TO_INTAKE_1(false, "SCORE_4_TO_INTAKE_1", PPPConstraints.AUTO_DEFAULT),
    SCORE_6_TO_INTAKE_4(false, "SCORE_6_TO_INTAKE_4", PPPConstraints.AUTO_DEFAULT),
    SCORE_7_TO_INTAKE_4(false, "SCORE_7_TO_INTAKE_4", PPPConstraints.AUTO_DEFAULT),
    SCORE_2_TO_INTAKE_2("SCORE_2_TO_INTAKE_2", PPPConstraints.AUTO_DEFAULT),
    INTAKE_3_TO_SCORE_7(false, "INTAKE_3_TO_SCORE_7", PPPConstraints.AUTO_DEFAULT);

    public final boolean currentlyExists;
    public final String name;
    public final PPPConstraints constraint;

    PathLegs(String name, PPPConstraints constraint) {
      this(true, name, constraint);
    }

    PathLegs(boolean exists, String name, PPPConstraints constraint) {
      this.currentlyExists = exists;
      this.name = name;
      this.constraint = constraint;
    }
  }
}
