package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
    FlywheelSim sim;
    PIDController pid;
    private double speedPoint;

    public ShooterIOSim() {
        sim = new FlywheelSim(DCMotor.getFalcon500(2), 1.5, 0.004);
        pid = new PIDController(0.0, 0.0, 0.0);
        speedPoint = 0.0;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        sim.update(0.02);

        inputs.leftShooterSpeed = sim.getAngularVelocityRPM() / 60;
        inputs.rightShooterSpeed = sim.getAngularVelocityRPM() / 60;
        inputs.shooterSpeedPoint = speedPoint;
        inputs.leftShooterAppliedVolts = sim.getCurrentDrawAmps();
        inputs.rightShooterAppliedVolts = sim.getCurrentDrawAmps();

    }

    @Override
    public void stop() {
        sim.setInputVoltage(0);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }

    @Override
    public void setSpeed(double rpm) {
        double appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public boolean nearSpeedPoint() {
        double motorSpeed = (sim.getAngularVelocityRPM() / 60);
        return Math.abs(motorSpeed - speedPoint) > Constants.Shooter.SHOOTER_MAX_SPEED_DEVIATION;
    }
}
