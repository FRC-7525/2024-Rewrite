package frc.robot.subsystems.AprilTagVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {

	private Optional<EstimatedRobotPose> frontBotpose3d;
	private Optional<EstimatedRobotPose> sideBotpose3d;

	public PhotonCamera frontCamera;
	public PhotonCamera sideCamera;
	private Drive driveSubsystem;

	private Transform3d frontRobotToCam;
	private Transform3d sideRobotToCam;

	private AprilTagFieldLayout layout;
	private PhotonPoseEstimator frontEstimator;
	private PhotonPoseEstimator sideEstimator;

	private boolean seesFrontVision;
	private boolean seesSideVision;

	private Timer frontVisionTimer;
	private Timer sideVisionTimer;

	public Vision(Drive driveSubsystem) {
		this.driveSubsystem = driveSubsystem;

		seesFrontVision = false;
		seesSideVision = false;

		frontVisionTimer = new Timer();
		sideVisionTimer = new Timer();

		sideCamera = new PhotonCamera("Side Camera");
		frontCamera = new PhotonCamera("Front Camera");

		sideRobotToCam = new Transform3d(
			new Translation3d(
				Units.inchesToMeters(-7.19),
				Units.inchesToMeters(11),
				Units.inchesToMeters(15.25)
			),
			new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(90))
		);
		frontRobotToCam = new Transform3d(
			new Translation3d(Units.inchesToMeters(-14.25), 0, Units.inchesToMeters(6)),
			new Rotation3d(0, Units.degreesToRadians(-67), Units.degreesToRadians(180))
		);

		// Cam layouts and stuff
		try {
			String deployDirectoryPath = Filesystem.getDeployDirectory().getAbsolutePath();
			layout = new AprilTagFieldLayout(deployDirectoryPath + "/CrescendoFieldLayout.json");
			// layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

			frontEstimator = new PhotonPoseEstimator(
				layout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				frontCamera,
				frontRobotToCam
			);
			sideEstimator = new PhotonPoseEstimator(
				layout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				sideCamera,
				sideRobotToCam
			);
		} catch (IOException e) {
			System.out.println(e);
		}
	}

	public void periodic() {
		frontBotpose3d = frontEstimator.update();
		sideBotpose3d = sideEstimator.update();
		SmartDashboard.putBoolean("Front Vision", seesFrontVision);
		SmartDashboard.putBoolean("Side Vision", seesSideVision);

		if (frontBotpose3d.isPresent()) {
			var tempPose = frontBotpose3d.get().estimatedPose;
			double[] frontPose = {
				tempPose.getX(),
				tempPose.getY(),
				Units.radiansToDegrees(tempPose.getRotation().getZ()),
			};

			SmartDashboard.putNumberArray("Front Pose", frontPose);
		}
		if (sideBotpose3d.isPresent()) {
			var sideTempPose = sideBotpose3d.get().estimatedPose;
			double[] sidePose = {
				sideTempPose.getX(),
				sideTempPose.getY(),
				Units.radiansToDegrees(sideTempPose.getRotation().getZ()),
			};

			SmartDashboard.putNumberArray("Side Pose", sidePose);
		}

		// Add Vision Measurments
		// System.out.println(getFrontPose2d().isPresent());
		// System.out.println(frontBotpose3d.isPresent());
		if (getFrontPose2d().isPresent()) {
			var frontPipeline = frontCamera.getLatestResult();
			driveSubsystem.addVisionMeasurement(
				getFrontPose2d().get(),
				Timer.getFPGATimestamp(),
				getEstimationStdDevs(getFrontPose2d().get(), frontPipeline)
			);
			// System.out.println("zzzzzzzzzzzzzzzz");
		}

		if (getSidePose2d().isPresent()) {
			var sidePipeline = sideCamera.getLatestResult();
			driveSubsystem.addVisionMeasurement(
				getSidePose2d().get(),
				Timer.getFPGATimestamp(),
				getEstimationStdDevs(getSidePose2d().get(), sidePipeline)
			);
		}
	}

	public Optional<Pose2d> getFrontPose2d() {
		if (frontBotpose3d.isPresent()) {
			seesFrontVision = true;
			frontVisionTimer.reset();
			frontVisionTimer.start();
			return Optional.of(frontBotpose3d.get().estimatedPose.toPose2d());
		} else {
			if (frontVisionTimer.get() > Constants.Vision.LAST_VISION_MEASURMENT_TIMER) {
				seesFrontVision = false;
			}
		}
		return Optional.empty();
	}

	public Optional<Pose2d> getSidePose2d() {
		if (sideBotpose3d.isPresent()) {
			seesSideVision = true;
			sideVisionTimer.reset();
			sideVisionTimer.start();

			return Optional.of(sideBotpose3d.get().estimatedPose.toPose2d());
		} else {
			if (sideVisionTimer.get() > Constants.Vision.LAST_VISION_MEASURMENT_TIMER) {
				seesSideVision = false;
			}
		}
		return Optional.empty();
	}

	public Matrix<N3, N1> getEstimationStdDevs(
		Pose2d estimatedPose,
		PhotonPipelineResult pipelineResult
	) {
		var estStdDevs = Constants.Vision.SINGLE_STD; // automatically assume one tag seen MIGHT BE SLOWER THAN GETTING
		// PIPELINE VALUE AND DECIDING????????
		var targets = pipelineResult.getTargets();
		int numTags = 0; // MIGHT BE SLOWER THAN GETTING PIPELINE VALUE AND DECIDING????????
		double avgDist = 0;
		double avgWeight = 0;
		for (var itag : targets) { // goes through all tags and calculates distance away from the target to bot
			var tagPose = layout.getTagPose(itag.getFiducialId());
			if (tagPose.isEmpty()) continue;
			numTags++;
			avgDist +=
			tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation()); // distance from bot to what tag should be

			avgWeight += Constants.Vision.TAG_WEIGHTS[itag.getFiducialId() - 1];
		}
		if (numTags == 0) return estStdDevs; // if you don't see don't change/keep the normal one

		avgDist /= numTags; // making it an average
		avgWeight /= numTags; // making it an average

		if (numTags > 1) {
			estStdDevs = Constants.Vision.MULTI_STD; // more trust in vision if mutliple tags
		}
		if (numTags == 1 && avgDist > Constants.Vision.STD_TRUSTABLE_DISTANCE) {
			estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE); // less trust in vision
			// if one tag
		} else {
			estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist) / 30)); // random goofy numbers from other team? but
			// basically gets weight from distance so
			// distance important but we have consistent i
			// think
		}

		estStdDevs = estStdDevs.times(avgWeight); // dynamic portion where matrix is updated based on how confident we
		// are in the tags we can see

		return estStdDevs;
	}
}
