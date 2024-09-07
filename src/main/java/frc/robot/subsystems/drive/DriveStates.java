package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.subsystems.SubsystemStates;

public enum DriveStates implements SubsystemStates {
	REGULAR_DRIVE("Regular Drive", Constants.Drive.REGULAR_TM, Constants.Drive.REGULAR_RM),
	SLOW_MODE("Slow Mode", Constants.Drive.SLOW_TM, Constants.Drive.SLOW_RM),
	AUTO_ALIGN("Auto Aligning", Constants.Drive.AA_TM, Constants.Drive.AA_RM),
	SPEED_MAXXING("Speed Maxxing", Constants.Drive.FAST_TM, Constants.Drive.FAST_RM);

	// Someone make auto align so that state is useful please

	DriveStates(String stateString, double translationModifier, double rotationModifier) {
		this.stateString = stateString;
		this.translationModifier = translationModifier;
		this.rotationModifier = rotationModifier;
	}

	String stateString;
	double translationModifier;
	double rotationModifier;

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getTranslationModifier() {
		return translationModifier;
	}

	public double getRotationModifier() {
		return rotationModifier;
	}
}
