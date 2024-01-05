package frc.robot.constants.enums;

import edu.wpi.first.math.util.Units;

public enum ModuleModels {
  SDS_MK4_L1(ModuleTypes.SDS_MK4, ModuleDriveRatios.SDS_L1),
  SDS_MK4_L2(ModuleTypes.SDS_MK4, ModuleDriveRatios.SDS_L2),
  SDS_MK4_L3(ModuleTypes.SDS_MK4, ModuleDriveRatios.SDS_L3),
  SDS_MK4_L4(ModuleTypes.SDS_MK4, ModuleDriveRatios.SDS_L4),
  SDS_MK4I_L1(ModuleTypes.SDS_MK4I, ModuleDriveRatios.SDS_L1),
  SDS_MK4I_L2(ModuleTypes.SDS_MK4I, ModuleDriveRatios.SDS_L2),
  SDS_MK4I_L3(ModuleTypes.SDS_MK4I, ModuleDriveRatios.SDS_L3);

  public final ModuleTypes type;
  public final ModuleDriveRatios ratio;
  public final double driveRatio, steerRatio;
  public final double theoreticalMaxWheelSpeed;

  ModuleModels(ModuleTypes type, ModuleDriveRatios ratio) {
    this.type = type;
    this.ratio = ratio;
    this.driveRatio = ratio.driveRatio;
    this.steerRatio = type.steerRatio;
    // motor RPM --> motor RPS --> wheel RPS --> wheel MPS
    // TODO check this number, probably replace theoretical max with on blocks max
    this.theoreticalMaxWheelSpeed = (6380.0 / 60.0) * type.wheelCircumferenceM / ratio.driveRatio;
  }

  public enum ModuleTypes {
    SDS_MK4(false, false, new int[][] {{32, 15}, {60, 10}}),
    SDS_MK4I(false, true, new int[][] {{50, 14}, {60, 10}});
    // SWERVEX
    // SWERVENSTEER

    public final boolean invertDrive;
    public final boolean invertSteer;
    public final double steerRatio;
    public final double wheelDiameterM = Units.inchesToMeters(3.94);
    public final double wheelCircumferenceM = wheelDiameterM * Math.PI;

    ModuleTypes(boolean invertDrive, boolean invertSteer, int[][] gearPairs) {
      this.invertDrive = invertDrive;
      this.invertSteer = invertSteer;

      double ratio = 1.0;
      for (int i = 0; i < gearPairs.length; i++) {
        assert (gearPairs[i].length == 2);
        ratio = ratio * gearPairs[i][0] / gearPairs[i][1];
      }
      this.steerRatio = ratio;
    }
  }

  public enum ModuleDriveRatios {
    SDS_L1(new int[][] {{50, 14}, {19, 25}, {45, 15}}),
    SDS_L2(new int[][] {{50, 14}, {17, 27}, {45, 15}}),
    SDS_L3(new int[][] {{50, 14}, {16, 28}, {45, 15}}),
    SDS_L4(new int[][] {{48, 16}, {16, 28}, {45, 15}});

    public final double driveRatio;

    ModuleDriveRatios(int[][] gearPairs) {
      double ratio = 1.0;
      for (int i = 0; i < gearPairs.length; i++) {
        assert (gearPairs[i].length == 2);
        ratio = ratio * gearPairs[i][0] / gearPairs[i][1];
      }
      this.driveRatio = ratio;
    }
  }
}
