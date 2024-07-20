package frc.robot.subsystems.exampleSubsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.exampleSubsystem.ExampleIO.ExampleIOInputs;

public class ExampleIOSparkMax implements ExampleIO {

  CANSparkMax motor;
  RelativeEncoder encoder;
  PIDController controller;
  double appliedVolts;
  double speedPoint;

  public ExampleIOSparkMax() {
    motor = new CANSparkMax(0, MotorType.kBrushless);
    controller = new PIDController(0, 0, 0);
    encoder = motor.getEncoder();
    appliedVolts = 0.0;
    speedPoint = 0.0;

    // Makes get position and velocity return radians
    encoder.setPositionConversionFactor(Math.PI * 2);
    encoder.setVelocityConversionFactor(Math.PI * 2);
  }

  public void updateInputs(ExampleIOInputs inputs) {
    inputs.wheelAppliedVolts = appliedVolts;
    inputs.wheelSpeed = encoder.getVelocity();
    inputs.wheelSpeedPoint = speedPoint;
  }

  // Speedpoint in Radians
  public void setSpeed(Double speedPoint) {
    this.speedPoint = speedPoint;
    appliedVolts = controller.calculate(encoder.getPosition(), speedPoint);
    motor.setVoltage(appliedVolts);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void stop() {
    appliedVolts = 0.0;
    motor.stopMotor();
  }

  public void configurePID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
