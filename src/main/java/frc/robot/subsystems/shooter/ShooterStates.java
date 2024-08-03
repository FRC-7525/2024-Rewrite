package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.SubsystemStates;

public enum ShooterStates implements SubsystemStates {
	OFF(Constants.Shooter.OFF, "Shooter Off"),
	SHOOTING(Constants.Shooter.SHOOTING, "Shooting Full"),
	PASSING_AMP(Constants.Shooter.FEEDING_AMP, "Passing To Amp");

	private double speedPoint;
	private String stateString;

	ShooterStates(double speedPoint, String stateString) {
		this.speedPoint = speedPoint;
		this.stateString = stateString;
	}

	public double getMotorSpeedpoint() {
		return speedPoint;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}
