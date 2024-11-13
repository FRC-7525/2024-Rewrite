package frc.robot.subsystems.noteVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
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

	public double noteAngle() {
		PhotonPipelineResult lastResult = noteCam.getLatestResult();
		List<PhotonTrackedTarget> noteData = lastResult.targets;
		Rotation3d rotation3d = new Rotation3d(0, 0, 180);
		for (PhotonTrackedTarget t : noteData) {
			rotation3d = t.getBestCameraToTarget().getRotation();
		}
		return rotation3d.getAngle();
	}

	public void periodic() {
		var result = noteCam.getLatestResult();
		if (result.hasTargets()) {
			hasTarget = true;
		} else {
			hasTarget = false;
		}
		SmartDashboard.putNumber("Angle", noteAngle());
		Logger.recordOutput("NoteVision/hasTarget", hasTarget);
	}
}
