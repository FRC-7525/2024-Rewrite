package frc.robot.subsystems.vision;

import static java.lang.System.arraycopy;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOReal implements VisionIO {
  // Forward Camera
  private final PhotonCamera frontCam;
  private final PhotonPoseEstimator frontPhotonPoseEstimator;

  // Left Side Camera
  private final PhotonCamera sideCam;
  private final PhotonPoseEstimator sidePhotonPoseEstimator;

  private Pose3d[] poseArray = new Pose3d[3];
  private double[] timestampArray = new double[3];
  private double[] visionStdArray = new double[9];
  private double[] latencyArray = new double[3];
  private int count = 0;

  public VisionIOReal() {
    frontCam = new PhotonCamera("front");
    frontPhotonPoseEstimator = new PhotonPoseEstimator(
        Constants.Vision.aprilTagFieldLayout,
        MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCam,
        Constants.Vision.frontCamToRobot);

    // Side Camera
    sideCam = new PhotonCamera("side");
    sidePhotonPoseEstimator = new PhotonPoseEstimator(
        Constants.Vision.aprilTagFieldLayout,
        MULTI_TAG_PNP_ON_COPROCESSOR,
        sideCam,
        Constants.Vision.sideCamToRobot);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    getEstimatedPoseUpdates();
    inputs.visionPoses = poseArray;
    inputs.timestamps = timestampArray;
    inputs.visionStdDevs = visionStdArray;
    inputs.latency = latencyArray;
    count += 1;
    if (count % 500 == 0) {
      frontCam.takeOutputSnapshot();
      sideCam.takeOutputSnapshot();
    }
  }

  public void getEstimatedPoseUpdates() {
    Optional<EstimatedRobotPose> pose = frontPhotonPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[0] = estimatedRobotPose.estimatedPose;
          timestampArray[0] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs = getEstimationStdDevs(estimatedRobotPose, Constants.Vision.CameraResolution.HIGH_RES);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 0, 3);
          latencyArray[0] = frontCam.getLatestResult().getLatencyMillis() / 1.0e3;
        },
        () -> {
          poseArray[0] = new Pose3d();
          timestampArray[0] = 0.0;
          latencyArray[0] = 0.0;
        });
    pose = sidePhotonPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[1] = estimatedRobotPose.estimatedPose;
          timestampArray[1] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs = getEstimationStdDevs(estimatedRobotPose, Constants.Vision.CameraResolution.HIGH_RES);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 3, 3);
          latencyArray[1] = sideCam.getLatestResult().getLatencyMillis() / 1.0e3;
        },
        () -> {
          poseArray[1] = new Pose3d();
          timestampArray[1] = 0.0;
          latencyArray[1] = 0.0;
        });
  }

  public Pose2d getNotePose(Pose2d botPose2d) {
    double height = 10;
    PhotonPipelineResult lastResult = sideCam.getLatestResult();
    List<PhotonTrackedTarget> noteData = lastResult.targets;

    for (PhotonTrackedTarget t : noteData) {
      double yaw = Math.abs(Units.degreesToRadians(t.getYaw()));
      double pitch = Math.abs(Units.degreesToRadians(t.getPitch()));

      double xToBot = Math.sin(yaw) * (height / Math.tan(pitch));
      double yToBot = Math.cos(yaw) * (height / Math.tan(pitch));
      Pose2d notePose2d = new Pose2d(botPose2d.getX() + xToBot, botPose2d.getY() + yToBot, botPose2d.getRotation());
      return notePose2d;
    } return null;
  }
}