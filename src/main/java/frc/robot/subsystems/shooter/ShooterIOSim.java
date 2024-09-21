package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {

	FlywheelSim sim;
	BangBangController controller;
	private double speedPoint;
	private double appliedVolts;

	public ShooterIOSim() {
		sim = new FlywheelSim(
			DCMotor.getFalcon500(Constants.Shooter.NUM_MOTORS),
			Constants.Shooter.SHOOTER_GEARING,
			Constants.Shooter.SHOOTER_MOI
		);
		controller = new BangBangController();
		speedPoint = 0.0;
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		sim.update(Constants.SIM_UPDATE_TIME);

		inputs.leftShooterSpeed = sim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF;
		inputs.rightShooterSpeed = sim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF;
		inputs.shooterSpeedPoint = speedPoint;
	}

	@Override
	public void updateOutputs(ShooterIOOutputs outputs) {
		outputs.leftShooterAppliedVolts = appliedVolts;
		outputs.rightShooterAppliedVolts = appliedVolts;
	}

	@Override
	public void stop() {
		appliedVolts = 0;
		sim.setInputVoltage(appliedVolts);
	}

	@Override
	public void setSpeed(double rps) {
		speedPoint = rps;
		appliedVolts = controller.calculate(
			sim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF,
			rps
		);
		sim.setInputVoltage(appliedVolts);
	}

	@Override
	public boolean nearSpeedPoint() {
		return (
			Math.abs((sim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF) - speedPoint) >
			Constants.Shooter.ERROR_OF_MARGIN
		);
	}

	@Override
	public double getAverageSpeed() {
		return sim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF;
	}
}
