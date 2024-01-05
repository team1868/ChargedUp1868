package frc.robot.constants;

import edu.wpi.first.math.Vector;
import frc.robot.constants.enums.CameraConfigs;
import frc.robot.constants.enums.CameraSets;
import java.util.Arrays;

public enum VisionConfs {
  // Concept: camera configurations stay the same but sometimes we want to change the set of cameras
  // that are actually initialized and interacted with. During a season we will change teh CameraSet
  // as needed on they fly
  // COMP_BOT_CONF(CameraSets.CAMERA_0, new CameraConfigs[] {CameraConfigs.COMP_LEFT_CAMERA}),
  COMP_BOT_CONF(
      CameraSets.BOTH_CAMERAS,
      new CameraConfigs[] {CameraConfigs.COMP_LEFT_CAMERA, CameraConfigs.COMP_RIGHT_CAMERA}
  ),
  PRACTICE_BOT_CONF(CameraSets.CAMERA_0, new CameraConfigs[] {CameraConfigs.PRACTICE_LEFT_CAMERA}),
  SWERVE_BASE_CONF(CameraSets.CAMERA_0, new CameraConfigs[] {CameraConfigs.DRIVE_BASE_CAMERA}),
  TEST_CONF(CameraSets.NO_CAMERAS, new CameraConfigs[0]);

  public final CameraSets setup;
  public final CameraConfigs[] cameras = new CameraConfigs[MAX_NUM_CAMERAS];

  VisionConfs(CameraSets setup, CameraConfigs[] cameras) {
    this.setup = setup;

    // Ensures the id set of cameras are valid
    assert (setup.ids.length <= cameras.length);
    assert (setup.ids.length <= MAX_NUM_CAMERAS);

    // Read in ids and cameras to their proper id index
    int[] ids = new int[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      assert (cameras[i].id < MAX_NUM_CAMERAS);
      ids[i] = cameras[i].id;
      this.cameras[ids[i]] = cameras[i];

      // Ensure camera ids don't overlap with previous ids
      for (int j = 0; j < i; j++) {
        assert (ids[j] != ids[i]);
      }
    }

    // Ensure each camera id in the Camera set is represented
    // (allows us to switch camera sets but not change camera config arrays on the fly)
    Arrays.sort(ids);
    for (int i = 0; i < setup.ids.length; i++) {
      assert (Arrays.binarySearch(ids, setup.ids[i]) != 0);
    }
  }

  public CameraConfigs getCameraConfig(int id) {
    assert (cameras[id] != null);
    return cameras[id];
  }

  public static final int MAX_NUM_CAMERAS = 2;

  public static final int DRIVE_PIPELINE_INDEX = 5;

  public static final int PIPELINE_INDEX_3D = 0;
  public static final int PIPELINE_INDEX_2D = 1;

  public static final double AMBIGUITY_SCALE = 160.0;
  public static final double AMBIGUITY_CONSTANT = 0.2;

  public static final double X_DISTANCE_SCALE = 0.0153; // 0.0000447 * 3;
  public static final double X_DISTANCE_POW = 0.257; // 1.81;

  public static final double Y_DISTANCE_SCALE = 0.0372; // 0.000271 * 3;
  public static final double Y_DISTANCE_POW = 0.371; // 1.99;

  public static final double THETA_SCALE = 95.2;
  public static final double THETA_CONSTANT = 56.4;

  public static final double DEFAULT_X_DEV = 0.06;
  public static final double DEFAULT_Y_DEV = 0.24;
  public static final double DEFAULT_THETA_DEV = 180;

  public static final double SINGLE_TAG_MAX_DIST_M = 2;
}
