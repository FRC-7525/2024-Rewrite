package frc.robot.subsystems.climbers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {

	private DCMotorSim climberSim;

	private PIDController controller;

	private double climberSetpoint;

	private double appliedVolts;

	public ClimberIOSim() {
		// Lol I'm not gona tune the sim
		climberSim = new DCMotorSim(DCMotor.getNEO(Constants.Climber.NUM_MOTORS), Constants.Climber.GEARING, Constants.Climber.JKG_M_SQUARED);
		controller = new PIDController(0, 0, 0);
		climberSetpoint = 0;
		appliedVolts = 0;
	}

	public void updateInput(ClimberIOInputs inputs) {
		inputs.leftClimberSpeed = climberSim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF;
		inputs.rightClimberSpeed = climberSim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF;
		inputs.leftClimberPosition = climberSim.getAngularPositionRotations();
		inputs.rightClimberPosition = climberSim.getAngularPositionRotations();
		inputs.leftClimberSetpoint = climberSetpoint;
		inputs.rightClimberSetpoint = climberSetpoint;
	}

	public void updateOutputs(ClimberIOOutputs outputs) {
		outputs.leftClimberAppliedVoltage = appliedVolts;
		outputs.rightClimberAppliedVoltage = appliedVolts;
	}

	public void setSetpoints(double leftSetpoint, double rightSetpoint) {
		climberSetpoint = leftSetpoint;
		appliedVolts = controller.calculate(
			climberSim.getAngularPositionRotations(),
			climberSetpoint
		);
		climberSim.setInputVoltage(appliedVolts);
	}

	public void configurePID(double kP, double kI, double kD) {
		controller.setPID(kP, kI, kD);
	}

	public void stop() {
		climberSim.setInputVoltage(0);
	}

	public boolean nearSetpoints() {
		return (
			Math.abs(climberSim.getAngularPositionRotations() - climberSetpoint) <
			Constants.Climber.ERROR_OF_MARGIN
		);
	}

	public void zeroClimbers() {
		return;
	}

	public boolean climbersZeroed() {
		return true;
	}
}
