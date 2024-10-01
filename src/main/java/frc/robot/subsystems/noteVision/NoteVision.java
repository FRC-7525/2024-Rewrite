package frc.robot.subsystems.noteVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NoteVision {

	private final PhotonCamera noteCam;

	private boolean hasTarget;
	Pose2d pose = new Pose2d(); // for note vision testing

	public NoteVision() {
		noteCam = new PhotonCamera("Note Cam");
		hasTarget = false;
	}

	public Pose2d getNotePose(Pose2d botPose2d) {
		PhotonPipelineResult lastResult = noteCam.getLatestResult();
		List<PhotonTrackedTarget> noteData = lastResult.targets;
		for (PhotonTrackedTarget t : noteData) {
			Transform3d noteTransform = t.getBestCameraToTarget();
			Pose2d notePose2d = new Pose2d(
				botPose2d.getX() + noteTransform.getX(),
				botPose2d.getY() + noteTransform.getY(),
				botPose2d.getRotation()
			);
			return notePose2d;
		}
		return null;
	}

	public void periodic() {
		var result = noteCam.getLatestResult();
		if (result.hasTargets()) {
			hasTarget = true;
		} else {
			hasTarget = false;
		}
		SmartDashboard.putNumber("noteX", getNotePose(pose).getX()); // for note vision testing
		SmartDashboard.putNumber("noteY", getNotePose(pose).getY()); // for note vision testing

		Logger.recordOutput("NoteVision/hasTarget", hasTarget);
	}
}
