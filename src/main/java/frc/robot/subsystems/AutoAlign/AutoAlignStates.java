package frc.robot.subsystems.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemStates;

public enum AutoAlignStates implements SubsystemStates {
	SOURCE_SPEAKER(
		Constants.AutoAlign.redSourceSpeakerPose,
		Constants.AutoAlign.blueAmpSpeakerPose,
		"DRIVING TO SOURCE SPEAKER"
	),
	AMP_SPEAKER(
		Constants.AutoAlign.redAmpSpeakerPose,
		Constants.AutoAlign.blueSourceSpeakerPose,
		"DRIVING TO AMP SPEAKER"
	),
	AMP(Constants.AutoAlign.redAmpPose, Constants.AutoAlign.blueAmpPose, "DRIVING TO AMP"),
	NOTE(new Pose2d(), new Pose2d(), "DRIVING TO NOTE VISION POSE"),
	OFF(new Pose2d(), new Pose2d(), "OFF");

	private String stateString;
	private Pose2d targetPose2dRed;
	private Pose2d targetPose2dBlue;

	AutoAlignStates(Pose2d targetRed, Pose2d targetBlue, String stateString) {
		this.targetPose2dRed = targetRed;
		this.targetPose2dBlue = targetBlue;
		this.stateString = stateString;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public Pose2d getTargetPose2dRed() {
		return targetPose2dRed;
	}

	public Pose2d getTargetPose2dBlue() {
		return targetPose2dBlue;
	}
}
