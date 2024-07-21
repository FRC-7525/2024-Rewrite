package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  FlywheelSim sim;
  PIDController pid;

  public ShooterIOSim() {
    sim = new FlywheelSim(DCMotor.getFalcon500(2), 1.5, 0.004);
    pid = new PIDController(0.0, 0.0, 0.0);
  }

  private double appliedVolts = 0.0;
  public double speedPoint = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);

    sim.update(0.02);

    inputs.leftShooterSpeed = sim.getAngularVelocityRPM() / 60;
    inputs.rightShooterSpeed = sim.getAngularVelocityRPM() / 60;
    inputs.shooterAppliedVolts = appliedVolts;
    inputs.shooterSpeedPoint = speedPoint;
  }

  @Override
  public void stop() {}

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
