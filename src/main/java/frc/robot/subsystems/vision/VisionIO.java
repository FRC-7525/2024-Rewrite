package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Constants.Vision.CameraResolution;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public Pose3d[] visionPoses = List.of(new Pose3d(), new Pose3d()).toArray(new Pose3d[0]);
    public double[] timestamps = new double[2];
    public double[] latency = new double[2];
    public double[] visionStdDevs = new double[5];
    public boolean hasSideVision = false;
    public boolean hasFrontVision = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Update the reference pose of the vision system. Currently only used in sim. */
  public default void updatePose(Pose2d pose) {}

  /**
   * The standard deviations of the estimated poses from vision cameras, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  default Matrix<N3, N1> getEstimationStdDevs(
      EstimatedRobotPose estimatedPose, CameraResolution resolution) {
    var estStdDevs =
        switch (resolution) {
          case HIGH_RES -> Constants.Vision.highResSingleTagStdDev;
          case NORMAL -> Constants.Vision.normalSingleTagStdDev;
        };
    var targets = estimatedPose.targetsUsed;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = Constants.Vision.aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .minus(estimatedPose.estimatedPose.toPose2d())
              .getTranslation()
              .getNorm();
    }

    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1
        && avgDist
            > switch (resolution) {
              case HIGH_RES -> 8;
              case NORMAL -> 5;
            }) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs =
          switch (resolution) {
            case HIGH_RES -> Constants.Vision.highResMultiTagStdDev;
            case NORMAL -> Constants.Vision.normalMultiTagStdDev;
          };
    }
    // Increase std devs based on (average) distance
    if (numTags == 1
        && avgDist
            > switch (resolution) {
              case HIGH_RES -> 6;
              case NORMAL -> 4;
            }) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 20));
    }

    return estStdDevs;
  }
}
