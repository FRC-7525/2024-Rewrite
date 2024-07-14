package frc.robot.subsystems.exampleSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.exampleSubsystem.ExampleIO.ExampleIOInputs;

public class ExampleIOSim {
  FlywheelSim sim;
  PIDController controller;
  double appliedVolts;
  double speedPoint;

  public ExampleIOSim() {
    // These values are jibberish
    sim = new FlywheelSim(DCMotor.getKrakenX60(1), 1.5, 0.004);
    controller = new PIDController(0, 0, 0);
    speedPoint = 0.0;
    appliedVolts = 0.0;
  }

  public void updateInputs(ExampleIOInputs inputs) {
    inputs.wheelAppliedVolts = appliedVolts;
    // Wheel Speed in same units across IO Abstractions
    inputs.wheelSpeed = sim.getAngularVelocityRadPerSec() * 60;
    inputs.wheelSpeedPoint = speedPoint;
  }

  public void setSpeed(Double speedPoint) {
    this.speedPoint = speedPoint;
    // Wheel Speed in same units across IO Abstractions
    appliedVolts = controller.calculate(sim.getAngularVelocityRadPerSec() * 60, speedPoint);
    sim.setInputVoltage(appliedVolts);
  }

  public double getPosition() {
    /* Flywheel Sim has no get position method :(, its not that important thought
    you should only log 3d poses for pivoting subsystems (Intake and Amp Bar)
    because thats all we can really simulate */
    return 0.0;
  }

  public void stop() {
    appliedVolts = 0.0;
    sim.setInputVoltage(0);
  }

  public void configurePID(Double kP, Double kI, Double kD) {
    controller.setPID(kP, kI, kD);
  }
}
