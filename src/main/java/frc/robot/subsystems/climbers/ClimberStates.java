package frc.robot.subsystems.climbers;

import frc.robot.Constants;
import frc.robot.subsystems.SubsystemStates;

public enum ClimberStates implements SubsystemStates {
	OFF(Constants.Climber.IDLE, Constants.Climber.IDLE, "Off"),
	CLIMBING(Constants.Climber.CLIMBING, Constants.Climber.CLIMBING, "Climbing");

	private double leftSetpoint;
	private double rightSetpoint;
	private String stateString;

	ClimberStates(double leftSetpoint, double rightSetpoint, String stateString) {
		this.leftSetpoint = leftSetpoint;
		this.rightSetpoint = rightSetpoint;
		this.stateString = stateString;
	}

	public double getLeftSetpoint() {
		return leftSetpoint;
	}

	public double getRightSetpoint() {
		return rightSetpoint;
	}

	public String getStateString() {
		return stateString;
	}
}
