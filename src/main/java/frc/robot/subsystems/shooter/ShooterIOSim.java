package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {

  FlywheelSim sim;
  PIDController pid;
  private double speedPoint;
  private double appliedVolts;

  public ShooterIOSim() {
    sim =
        new FlywheelSim(
            DCMotor.getFalcon500(2),
            Constants.Shooter.SHOOTER_GEARING,
            Constants.Shooter.SHOOTER_MOI);
    pid = new PIDController(0.0, 0.0, 0.0);
    speedPoint = 0.0;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sim.update(Constants.SIM_UPDATE_TIME);

    inputs.leftShooterSpeed = sim.getAngularVelocityRPM() / 60;
    inputs.rightShooterSpeed = sim.getAngularVelocityRPM() / 60;
    inputs.shooterSpeedPoint = speedPoint;
    inputs.leftShooterAppliedVolts = appliedVolts;
    inputs.rightShooterAppliedVolts = appliedVolts;
  }

  @Override
  public void stop() {
    appliedVolts = 0;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  @Override
  public void setSpeed(double rps) {
    speedPoint = rps;
    appliedVolts = pid.calculate(sim.getAngularVelocityRPM() / 60, rps);
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public boolean nearSpeedPoint() {
    return Math.abs((sim.getAngularVelocityRPM() / 60) - speedPoint)
        > Constants.Shooter.ERROR_OF_MARGIN;
  }


	FlywheelSim sim;
	PIDController pid;
	private double speedPoint;
	private double appliedVolts;

	public ShooterIOSim() {
		sim = new FlywheelSim(
			DCMotor.getFalcon500(Constants.Shooter.NUM_MOTORS),
			Constants.Shooter.SHOOTER_GEARING,
			Constants.Shooter.SHOOTER_MOI
		);
		pid = new PIDController(0.0, 0.0, 0.0);
		speedPoint = 0.0;
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		sim.update(Constants.SIM_UPDATE_TIME);

		inputs.leftShooterSpeed = sim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF;
		inputs.rightShooterSpeed = sim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF;
		inputs.shooterSpeedPoint = speedPoint;
		inputs.leftShooterAppliedVolts = appliedVolts;
		inputs.rightShooterAppliedVolts = appliedVolts;
	}

	@Override
	public void stop() {
		appliedVolts = 0;
		sim.setInputVoltage(appliedVolts);
	}

	@Override
	public void configurePID(double kP, double kI, double kD) {
		pid.setPID(kP, kI, kD);
	}

	@Override
	public void setSpeed(double rps) {
		speedPoint = rps;
		appliedVolts = pid.calculate(sim.getAngularVelocityRPM() / Constants.RPM_TO_RPS_CF, rps);
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
