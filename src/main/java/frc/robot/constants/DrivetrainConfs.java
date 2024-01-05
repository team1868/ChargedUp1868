package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.constants.enums.*;

public enum DrivetrainConfs {
  COMP_BOT_CONFS(
      DrivetrainDimensions.COMP_BOT_DIMENSIONS,
      DrivetrainPorts.COMP_PORTS,
      DrivetrainControl.COMP_CONTROL,
      4,
      ModuleModels.SDS_MK4I_L2,
      ModuleControl.FALCON_MK4I_FALCON_L2,
      new double[] {360.0 - 186.38, 360.0 - 91.41, 360.0 - 199.27, 360.0 - 52.54}
  ),
  PRACTICE_BOT_CONFS(
      DrivetrainDimensions.PRACTICE_BOT_DIMENSIONS,
      DrivetrainPorts.PRACTICE_PORTS,
      DrivetrainControl.PRACTICE_CONTROL,
      4,
      ModuleModels.SDS_MK4I_L2,
      ModuleControl.FALCON_MK4I_FALCON_L2,
      new double[] {
          359.64,
          208.98,
          291.9,
          77.42,
      }
  ),
  SWERVE_BASE_CONFS(
      DrivetrainDimensions.SWERVE_BASE_DIMENSIONS,
      DrivetrainPorts.SWERVE_BASE_PORTS,
      DrivetrainControl.SWERVE_BASE_CONTROL,
      4,
      ModuleModels.SDS_MK4_L2,
      ModuleControl.FALCON_MK4_FALCON_L2,
      new double[] {360.0 - 52.9, 360.0 - 7.5, 360.0 - 21.8, 360.0 - 349.0}
      // new double[] {11.4, 338.1, 212.4, 81.5}
  );

  // Drivetrain Configurations
  public final DrivetrainDimensions dims;
  public final DrivetrainPorts ports;
  public final DrivetrainControl control;

  /* Module Configurations */
  public final ModuleModels model;
  public final ModuleControl moduleControl;
  public final int numModules;
  public final double[] moduleOffsetsDeg;
  public final ModuleModels.ModuleTypes type;
  public final ModuleModels.ModuleDriveRatios ratio;
  public final double alignmentToleranceDeg;
  public final double theoreticalMaxTranslationSpeed;
  public final double theoreticalMaxRotationalSpeed;

  DrivetrainConfs(
      DrivetrainDimensions dims,
      DrivetrainPorts ports,
      DrivetrainControl control,
      int numModules,
      ModuleModels model,
      ModuleControl moduleControl,
      double[] offsets
  ) {
    this(dims, ports, control, numModules, model, moduleControl, offsets, 5.0);
  }

  DrivetrainConfs(
      DrivetrainDimensions dims,
      DrivetrainPorts ports,
      DrivetrainControl control,
      int numModules,
      ModuleModels model,
      ModuleControl moduleControl,
      double[] offsets,
      double alignmentToleranceDeg
  ) {
    assert (numModules == ports.drive.length);
    assert (numModules == ports.steer.length);
    assert (numModules == ports.encoder.length);

    this.dims = dims;
    this.ports = ports;
    this.control = control;
    this.numModules = numModules;
    this.model = model;
    this.moduleControl = moduleControl;
    this.type = model.type;
    this.ratio = model.ratio;
    this.alignmentToleranceDeg = alignmentToleranceDeg;

    assert (offsets.length == numModules);
    moduleOffsetsDeg = offsets;

    this.theoreticalMaxTranslationSpeed = model.theoreticalMaxWheelSpeed;

    double location_m = Math.hypot(dims.halfTrackLength_M, dims.halfTrackWidth_M);
    this.theoreticalMaxRotationalSpeed = 2 * Math.PI * location_m / model.theoreticalMaxWheelSpeed;
  };
}
