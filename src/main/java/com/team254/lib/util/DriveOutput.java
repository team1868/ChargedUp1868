package com.team254.lib.util;

import com.team254.lib.geometry.Rotation2d254;
import com.team254.lib.physics.SwerveDrive;
import com.team254.lib.swerve.SwerveModuleState;

/**
 * Represents a closed loop output to the drivebase
 */
public class DriveOutput {
  public Rotation2d254[] azi_positions; // rad
  public double[] drive_vels; // m/s

  public DriveOutput() {
    this(
        new Rotation2d254[] {
            Rotation2d254.identity(),
            Rotation2d254.identity(),
            Rotation2d254.identity(),
            Rotation2d254.identity()},
        new double[] {0, 0, 0, 0}
    );
  }

  public DriveOutput(Rotation2d254[] azi_positions, double[] drive_vels) {
    this.azi_positions = azi_positions;
    this.drive_vels = drive_vels;
  }

  public static DriveOutput fromSwerveModuleStates(SwerveModuleState[] state) {
    double[] drive = new double[4];
    Rotation2d254[] azi = new Rotation2d254[4];
    for (int i = 0; i < 4; i++) {
      drive[i] = state[i].speedMetersPerSecond;
      azi[i] = state[i].angle;
    }
    return new DriveOutput(azi, drive);
  }
}
