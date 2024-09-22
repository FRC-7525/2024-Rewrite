package frc.robot.subsystems.noteVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NoteVision {

	private final PhotonCamera noteCam;

	private boolean hasTarget;

	public NoteVision() {
		noteCam = new PhotonCamera("Note Cam");
		hasTarget = false;
	}

	public Pose2d getNotePose(Pose2d botPose2d) {
		double height = 22; // need to confirm
		PhotonPipelineResult lastResult = noteCam.getLatestResult();
		List<PhotonTrackedTarget> noteData = lastResult.targets;
		for (PhotonTrackedTarget t : noteData) {
			double yaw = Math.abs(Units.degreesToRadians(t.getYaw()));
			double pitch = Math.abs(Units.degreesToRadians(t.getPitch()));
			double xToBot = Math.sin(yaw) * (height / Math.tan(pitch));
			double yToBot = Math.cos(yaw) * (height / Math.tan(pitch));
			Pose2d notePose2d = new Pose2d(
				botPose2d.getX() + xToBot,
				botPose2d.getY() + yToBot,
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
		// Sounds like an output??
		Logger.recordOutput("NoteVision/hasTarget", hasTarget);
	}
}
