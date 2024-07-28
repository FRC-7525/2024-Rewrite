package frc.robot.subsystems.vision;

import static java.lang.System.arraycopy;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
  private final VisionSystemSim visionSim;

  // Forward Camera
  private final PhotonCameraSim frontCam;
  private final PhotonPoseEstimator frontPhotonPoseEstimator;

  // Side Camera
  private final PhotonCameraSim sideCam;
  private final PhotonPoseEstimator sidePhotonPoseEstimator;

  private Pose3d[] poseArray = new Pose3d[2];
  private double[] timestampArray = new double[2];
  private double[] visionStdArray = new double[6];
  private boolean hasFrontVision = false;
  private boolean hasSideVision = false;

  // Hahahha you cant stop me from making more timers, I win
  private Timer sideVisionTimer = new Timer();
  private Timer frontVisionTimer = new Timer();

  public VisionIOSim() {
    PhotonCamera front = new PhotonCamera("front");
    PhotonCamera side = new PhotonCamera("side");
    frontVisionTimer.start();
    sideVisionTimer.start();

    frontPhotonPoseEstimator =
        new PhotonPoseEstimator(
            Constants.Vision.aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            front,
            Constants.Vision.frontCamToRobot);
    sidePhotonPoseEstimator =
        new PhotonPoseEstimator(
            Constants.Vision.aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            side,
            Constants.Vision.sideCamToRobot);

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(Constants.Vision.aprilTagFieldLayout);

    // TODO: Get accurate numbers & put these in constants
    SimCameraProperties frontCameraProp = new SimCameraProperties();
    frontCameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(95.95));
    frontCameraProp.setCalibError(0.25, 0.10);
    frontCameraProp.setFPS(25);
    frontCameraProp.setAvgLatencyMs(50);
    frontCameraProp.setLatencyStdDevMs(15);

    SimCameraProperties sideCameraProp = new SimCameraProperties(); // Arducam OV9281, not Spinel
    sideCameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(84.47));
    sideCameraProp.setCalibError(0.25, 0.10);
    sideCameraProp.setFPS(40);
    sideCameraProp.setAvgLatencyMs(40);
    sideCameraProp.setLatencyStdDevMs(10);

    frontCam = new PhotonCameraSim(front, frontCameraProp);
    sideCam = new PhotonCameraSim(side, sideCameraProp);

    visionSim.addCamera(frontCam, Constants.Vision.frontCamToRobot);
    visionSim.addCamera(sideCam, Constants.Vision.sideCamToRobot);

    frontCam.enableDrawWireframe(true);
    sideCam.enableDrawWireframe(true);

    // Set things to stuff because becauseb before it would just spam null pointer
    for (int i = 0; i < poseArray.length; i++) {
      poseArray[i] = new Pose3d();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    getEstimatedPoseUpdates();
    inputs.visionPoses = poseArray;
    inputs.timestamps = timestampArray;
    inputs.visionStdDevs = visionStdArray;
    inputs.hasFrontVision = hasFrontVision;
    inputs.hasSideVision = hasSideVision;
  }

  public void updatePose(Pose2d pose) {
    visionSim.update(pose);
  }

  public void getEstimatedPoseUpdates() {
    Optional<EstimatedRobotPose> pose = frontPhotonPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          hasFrontVision = true;
          poseArray[0] = estimatedRobotPose.estimatedPose;
          timestampArray[0] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs =
              getEstimationStdDevs(estimatedRobotPose, Constants.Vision.CameraResolution.HIGH_RES);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 0, 3);
          frontVisionTimer.reset();
        },
        () -> {
          if (frontVisionTimer.get() > Constants.Vision.LOST_VISION_THRESHOLD) {
            hasFrontVision = false;
          }
        });

    pose = sidePhotonPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          hasSideVision = true;
          poseArray[1] = estimatedRobotPose.estimatedPose;
          timestampArray[1] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs =
              getEstimationStdDevs(estimatedRobotPose, Constants.Vision.CameraResolution.NORMAL);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 3, 3);
          sideVisionTimer.restart();
        },
        () -> {
          if (sideVisionTimer.get() > Constants.Vision.LOST_VISION_THRESHOLD) {
            hasSideVision = false;
          }
        });
  }
}
