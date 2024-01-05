package frc.robot.utils;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.VisionConfs;
import frc.robot.constants.enums.CameraConfigs;
import frc.robot.constants.enums.CameraSets;
import frc.robot.subsystems.VisionController;

public class MultiCameraController {
  private static final VisionConfs VISION_CONSTS = Constants.CRobot.vision;
  private static final CameraSets _configuredCameras = VISION_CONSTS.setup;
  ;
  // When we set the desired camera set, make sure that it's a valid subset of the initialized
  // camera set;
  CameraSets _desiredCameras = _configuredCameras;
  private VisionController[] _cameras = new VisionController[VisionConfs.MAX_NUM_CAMERAS];
  private double[] _lastTimestamp = new double[VisionConfs.MAX_NUM_CAMERAS];
  private FieldObject2d[] _visionFieldPoses = new FieldObject2d[VisionConfs.MAX_NUM_CAMERAS];
  private Pose2d[] _visionPoses = new Pose2d[VisionConfs.MAX_NUM_CAMERAS];
  private SwerveDrivePoseEstimator _robotPoseEstimator;

  public MultiCameraController(Field2d field, SwerveDrivePoseEstimator robotPoseEstimator) {
    _robotPoseEstimator = robotPoseEstimator;
    // For each id in the camera set, configure the camera
    for (int i = 0; i < VISION_CONSTS.setup.ids.length; i++) {
      // Camera id array and camera config array are sorted and parallel with a separate tracking
      // value for primary camera to remove extra computation from this stage
      int id = VISION_CONSTS.setup.ids[i];
      CameraConfigs conf = VISION_CONSTS.getCameraConfig(id);

      // Initialize the camera, each camera knows which ID it is -- ensuring we can have continuity
      // when deciding to ignore a camera while testing or debugging
      _cameras[id] = new VisionController(conf.cameraName, conf.r2cTransform);
      _visionFieldPoses[id] = field.getObject("Vision Pose " + conf.fieldObjectName);
      _lastTimestamp[i] = -1.0;
    }

    for (int i = 0; i < VISION_CONSTS.MAX_NUM_CAMERAS; i++) {
      _visionPoses[i] = new Pose2d();
    }
  }

  public void updateOdom(boolean isAuto) {
    if (RobotAltModes.isVisionMode && !isAuto) {
      for (int i = 0; i < _desiredCameras.ids.length; i++) {
        updateOdom(_desiredCameras.ids[i]);
      }
    }
  }

  private void updateOdom(int id) {
    double timestamp = _cameras[id].getLatestTimestamp(_visionPoses[id]);
    if (_lastTimestamp[id] < timestamp) {
      _lastTimestamp[id] = timestamp;
      _visionPoses[id] = _cameras[id].getLatestPose();
      _robotPoseEstimator.addVisionMeasurement(
          _visionPoses[id], _lastTimestamp[id], _cameras[id].getLatestStdDevs()
      );
    }
  }

  // Gets vision pose for the camera with the passed id
  public Pose2d getVisionPose(int id) {
    if (id < VisionConfs.MAX_NUM_CAMERAS && id >= 0 && _cameras[id] != null)
      return _visionPoses[id];

    return new Pose2d();
  }

  // Gets vision pose of the primary camera
  public Pose2d getVisionPose() {
    // if (_visionPoses[_desiredCameras.primary] == null)
    //   return new Pose2d();
    return getVisionPose(_desiredCameras.primary);
  }

  public void setDesiredCameras(CameraSets set) {
    if (_configuredCameras.isCompatible(set)) {
      _desiredCameras = set;
    }
  }

  public void updateShuffleboard() {
    for (int i = 0; i < VisionConfs.MAX_NUM_CAMERAS; i++) {
      _visionFieldPoses[i].setPose(getVisionPose(_desiredCameras.ids[i]));
    }
  }

  public CameraSets getDesiredCameras() {
    return _desiredCameras;
  }
}
