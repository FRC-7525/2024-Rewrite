package frc.robot.subsystems.noteVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class NoteVisionIOReal implements NoteVisionIO{
private final PhotonCamera noteCam;

  public NoteVisionIOReal() {
    noteCam = new PhotonCamera("Note Cam");
  }

  public void updateInputs(NoteVisionIOInputs inputs) {
    var result = noteCam.getLatestResult();
    if (result.hasTargets()) {
      inputs.hasTarget = true;
    } else {
      inputs.hasTarget = false;
    }
}
public Pose2d getNotePose(Pose2d botPose2d) {
    double height = 10;

    PhotonPipelineResult lastResult = noteCam.getLatestResult();
    List<PhotonTrackedTarget> noteData = lastResult.targets;

    for (PhotonTrackedTarget t : noteData) {
      double yaw = Math.abs(Units.degreesToRadians(t.getYaw()));
      double pitch = Math.abs(Units.degreesToRadians(t.getPitch()));

      double xToBot = Math.sin(yaw) * (height / Math.tan(pitch));
      double yToBot = Math.cos(yaw) * (height / Math.tan(pitch));
      Pose2d notePose2d =
          new Pose2d(botPose2d.getX() + xToBot, botPose2d.getY() + yToBot, botPose2d.getRotation());
      return notePose2d;
    } return null;
}
}
