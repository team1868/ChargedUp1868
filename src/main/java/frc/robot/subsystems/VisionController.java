package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.VisionConfs;
import frc.robot.utils.LoopTimer;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionController extends SubsystemBase {
  private Matrix<N3, N1> _stdDevs;
  private Pose2d _latestPose = new Pose2d();

  GenericEntry _photonDistanceEntry;
  GenericEntry _photonYawEntry;
  GenericEntry _photonPitchEntry;

  private static final AprilTagFieldLayout _aprilTagFieldLayout = new AprilTagFieldLayout(
      Constants.CField.aprilTags, Constants.CField.dims.x_M, Constants.CField.dims.y_M
  );

  // Cameras: "OV5647" "Microsoft_LifeCam_HD-3000"
  // Cameras: "Arducam_Forward", "Arducam_Backward"
  private String _cameraName;
  private PhotonCamera _camera;
  private Transform3d _robotToCamera;
  private Transform3d _multiTagRobotToCamera;
  private PhotonPoseEstimator _visionPoseEstimator;

  private boolean _usingMultiTagTransform = false;
  private Matrix<N3, N1> _maxDevs =
      VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private Timer timer = new Timer();

  public VisionController(String cameraName, Transform3d robotToCamera) {
    _cameraName = cameraName;
    _robotToCamera = robotToCamera;

    // Has to happen after the cameraName and _robotToCamera is given a value
    _camera = new PhotonCamera(_cameraName);
    _multiTagRobotToCamera = new Transform3d(
        _robotToCamera.getTranslation(),
        // This may be C++ specific and we can just do
        // _robotToCamera.getRotation();
        new Rotation3d(
            _robotToCamera.getRotation().getX(),
            -_robotToCamera.getRotation().getY(),
            _robotToCamera.getRotation().getZ()
        )
    );
    _visionPoseEstimator = new PhotonPoseEstimator(
        _aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, _camera, _robotToCamera
    );
  }

  public VisionController() {
    this("", new Transform3d());
  }

  public EstimatedRobotPose getVisionPose3d() {
    var result = _camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();

    boolean redTarget = false;
    boolean blueTarget = false;

    for (final PhotonTrackedTarget target : targets) {
      int id = target.getFiducialId();
      if (id > 8 || id < 1) {
        // illegal tag
        return null;
      }
      if (id < 5) {
        if (blueTarget) {
          // Invalid tag combo
          return null;
        }
        redTarget = true;
      } else {
        // Invalid tag combo
        if (redTarget) {
          return null;
        }
        blueTarget = true;
      }
    }

    // return _visionPoseEstimator.update().orElse(null);
    // ^^ Original, C++ could not reuse the result from earlier
    return _visionPoseEstimator.update(result).orElse(null);
  }

  public double updateVisionPoseStdDevs(Pose2d pose) {
    EstimatedRobotPose result = getVisionPose3d();

    LoopTimer.markEvent("VISION 0");

    if (result == null) {
      _latestPose = pose;
      _stdDevs = _maxDevs;
      return 0.0;
    }

    // System.out.println("----VC" + result.estimatedPose.getX());
    // System.out.println("----VC" + result.estimatedPose.getY());

    int targetsUsedSize = result.targetsUsed.size();

    LoopTimer.markEvent("VISION 0.5");

    if (targetsUsedSize > 1 && !_usingMultiTagTransform) {
      _visionPoseEstimator.setRobotToCameraTransform(_multiTagRobotToCamera);
      _usingMultiTagTransform = true;

      _latestPose = pose;
      _stdDevs = _maxDevs;
      return 0.0;
    } else if (targetsUsedSize == 1 && _usingMultiTagTransform) {
      _visionPoseEstimator.setRobotToCameraTransform(_robotToCamera);
      _usingMultiTagTransform = false;

      _latestPose = pose;
      _stdDevs = _maxDevs;
      return 0.0;
    }

    LoopTimer.markEvent("VISION 1");

    double smallestDistance = Double.MAX_VALUE;

    for (PhotonTrackedTarget target : result.targetsUsed) {
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      if (distance < smallestDistance) {
        smallestDistance = distance;
      }
    }

    if (smallestDistance > Constants.CField.dims.halfX_M
        || (!_usingMultiTagTransform && smallestDistance > VisionConfs.SINGLE_TAG_MAX_DIST_M)) {
      _latestPose = pose;
      _stdDevs = _maxDevs;
      return 0.0;
    }

    LoopTimer.markEvent("VISION 2");
    double ambiguity = targetsUsedSize != 1
        ? 1.0
        : VisionConfs.AMBIGUITY_SCALE * result.targetsUsed.get(0).getPoseAmbiguity()
            + VisionConfs.AMBIGUITY_CONSTANT;

    double xDev = VisionConfs.X_DISTANCE_SCALE
        * Math.exp(VisionConfs.X_DISTANCE_POW * smallestDistance) * ambiguity;
    double yDev = VisionConfs.Y_DISTANCE_SCALE
        * Math.exp(VisionConfs.Y_DISTANCE_POW * smallestDistance) * ambiguity;
    double thetaDev =
        VisionConfs.THETA_SCALE * Math.log(smallestDistance) * VisionConfs.THETA_CONSTANT;

    LoopTimer.markEvent("VISION 3");

    _latestPose = result.estimatedPose.toPose2d();
    _stdDevs = VecBuilder.fill(xDev, yDev, thetaDev);

    return result.timestampSeconds;
  }

  public Pose2d getLatestPose() {
    return _latestPose;
  }

  public double getLatestTimestamp(Pose2d pose) {
    return updateVisionPoseStdDevs(pose);
  }

  public Matrix<N3, N1> getLatestStdDevs() {
    return _stdDevs;
  }

  public Matrix<N3, N1> getMaxDevs() {
    return _maxDevs;
  }

  public void setDriverMode() {
    _camera.setDriverMode(true);
  }

  public void set3dMode() {
    if (_camera.getPipelineIndex() != VisionConfs.PIPELINE_INDEX_3D) {
      _camera.setPipelineIndex(VisionConfs.PIPELINE_INDEX_3D);
    }
  }

  public void set2dMode() {
    if (_camera.getPipelineIndex() != VisionConfs.PIPELINE_INDEX_2D) {
      _camera.setPipelineIndex(VisionConfs.PIPELINE_INDEX_2D);
    }
  }

  public void configShuffleboard() {
    ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
    ShuffleboardLayout photonLayout =
        drivetrainTab.getLayout("Photon " + _cameraName, BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withPosition(2, 0);
    _photonDistanceEntry = photonLayout.add("Photon " + _cameraName + " Distance", 0.0)
                               .withSize(2, 1)
                               .withPosition(0, 0)
                               .getEntry();
    _photonYawEntry = photonLayout.add("Photon " + _cameraName + " Yaw", 0.0)
                          .withSize(2, 1)
                          .withPosition(0, 1)
                          .getEntry();
    _photonPitchEntry = photonLayout.add("Photon " + _cameraName + " Pitch", 0.0)
                            .withSize(2, 1)
                            .withPosition(0, 2)
                            .getEntry();
  }

  public void updateShuffleboard(double yaw, double distance, double pitch) {
    _photonPitchEntry.setDouble(pitch);
    _photonYawEntry.setDouble(yaw);
    _photonDistanceEntry.setDouble(distance);
  }
}
