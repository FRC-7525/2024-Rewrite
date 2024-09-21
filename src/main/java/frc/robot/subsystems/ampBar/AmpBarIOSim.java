package frc.robot.subsystems.ampBar;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class AmpBarIOSim implements AmpBarIO {

	SingleJointedArmSim pivotSim;
	DCMotorSim spinnerSim;
	PIDController controller;

	String stateString;

	double pivotAppliedVoltage;
	double pivotSetpoint;

	double spinnerAppliedVoltage;
	double spinnerSpeedpoint;

	public AmpBarIOSim() {
		// the sim values are random
		pivotSim = new SingleJointedArmSim(
			DCMotor.getNEO(Constants.AmpBar.NUM_PIVOT_MOTORS),
			Constants.AmpBar.PIVOT_GEARING,
			Constants.AmpBar.PIVOT_MOI,
			Constants.AmpBar.PIVOT_LENGTH_METERS,
			Constants.AmpBar.MIN_PIVOT_POSITION,
			Constants.AmpBar.MAX_PIVOT_POSITION,
			false,
			0
		);
		spinnerSim = new DCMotorSim(
			DCMotor.getKrakenX60(Constants.AmpBar.NUM_SPINNER_MOTORS),
			Constants.AmpBar.SPINNER_GEARING,
			Constants.AmpBar.SPINNER_MOI
		);

		controller = new PIDController(0, 0, 0);
		stateString = "";

		pivotAppliedVoltage = 0;
		pivotSetpoint = 0;
		spinnerAppliedVoltage = 0;
		spinnerSpeedpoint = 0;
	}

	@Override
	public void updateInput(AmpBarIOInputs inputs) {
		inputs.ampBarState = stateString;

		inputs.pivotPosition = getPivotPosition();
		inputs.pivotSetpoint = pivotSetpoint;

		inputs.spinnerSpeed = getSpinnerSpeed();
		inputs.spinnerSetpoint = spinnerSpeedpoint;

		pivotSim.update(Constants.SIM_UPDATE_TIME);
		spinnerSim.update(Constants.SIM_UPDATE_TIME);
	}

	@Override
	public void updateOutputs(AmpBarIOOutputs outputs) {
		outputs.spinnerAppliedVoltage = spinnerAppliedVoltage;
		outputs.pivotAppliedVoltage = pivotAppliedVoltage;
	}

	@Override
	public void setPivotPosition(double position) {
		pivotSetpoint = position;
		pivotAppliedVoltage = controller.calculate(pivotSim.getAngleRads(), pivotSetpoint);
		pivotSim.setInputVoltage(pivotAppliedVoltage);
	}

	@Override
	public void setSpinnerSpeedpoint(double speed) {
		spinnerSpeedpoint = speed;
		spinnerAppliedVoltage = speed;
		spinnerSim.setInputVoltage(speed);
	}

	@Override
	public void stop() {
		pivotAppliedVoltage = 0;
		spinnerAppliedVoltage = 0;
		spinnerSim.setInputVoltage(0);
		pivotSim.setInputVoltage(0);
	}

	@Override
	public double getPivotPosition() {
		return pivotSim.getAngleRads();
	}

	@Override
	public double getSpinnerSpeed() {
		return spinnerSim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF;
	}

	@Override
	public void configurePID(double kP, double kI, double kD) {
		controller.setPID(kP, kI, kD);
	}

	@Override
	public boolean nearSetPoint() {
		double motorPosition = getPivotPosition();
		return Math.abs(motorPosition - pivotSetpoint) <= Constants.AmpBar.ERROR_OF_MARGIN;
	}

	@Override
	public boolean nearSpeedPoint() {
		return Math.abs(getSpinnerSpeed() - spinnerSpeedpoint) <= Constants.AmpBar.ERROR_OF_MARGIN;
	}
}
