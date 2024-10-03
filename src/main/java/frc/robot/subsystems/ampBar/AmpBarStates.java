package frc.robot.subsystems.ampBar;

import frc.robot.Constants;
import frc.robot.subsystems.SubsystemStates;

public enum AmpBarStates implements SubsystemStates {
	OFF(Constants.AmpBar.IN, Constants.AmpBar.OFF, "Amp Bar Off"),
	SHOOTING(Constants.AmpBar.OUT, Constants.AmpBar.SHOOTING, "Shooting Amp"),
	FEEDING(Constants.AmpBar.FEEDING_POSITION, Constants.AmpBar.FEEDING, "Getting Fed"),
	HOLDING_NOTE(Constants.AmpBar.OUT, Constants.AmpBar.OFF, "Holding a Note"),
	OUT_FOR_CLIMBER(Constants.AmpBar.OUT, Constants.AmpBar.OFF, "Out for Climber"),
	CLEAR_INTAKE_CAM(-29.3, Constants.AmpBar.OFF, "Out of Way for Cam");

	private double pivotPositionSetpoint;
	private double spinnerMotorSpeedpoint;
	private String stateString;

	AmpBarStates(double pivotMotorSetpoint, double spinnerMotorSpeedpoint, String stateString) {
		this.pivotPositionSetpoint = pivotMotorSetpoint;
		this.spinnerMotorSpeedpoint = spinnerMotorSpeedpoint;
		this.stateString = stateString;
	}

	public double getPivotPositionSetpoint() {
		return pivotPositionSetpoint;
	}

	public double getMotorSpeedpoint() {
		return spinnerMotorSpeedpoint;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}
