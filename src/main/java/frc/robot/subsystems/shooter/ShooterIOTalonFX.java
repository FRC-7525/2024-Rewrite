package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private PIDController feedbackController;
  private double speedPoint;

  public ShooterIOTalonFX() {
    feedbackController = new PIDController(0, 0, 0);
    leftMotor = new TalonFX(15);
    rightMotor = new TalonFX(14);
    speedPoint = 0.0;
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftShooterAppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightShooterAppliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftShooterSpeed = leftMotor.getVelocity().getValueAsDouble();
    inputs.rightShooterSpeed = rightMotor.getVelocity().getValueAsDouble();
    inputs.shooterSpeedPoint = speedPoint;
  }

  public void setSpeed(double rps) {
    speedPoint = rps;
    double left =
        feedbackController.calculate(leftMotor.getRotorVelocity().getValueAsDouble(), rps);
    double right =
        feedbackController.calculate(rightMotor.getRotorVelocity().getValueAsDouble(), rps);

    leftMotor.setVoltage(left);
    rightMotor.setVoltage(right);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void configurePID(double kP, double kI, double kD) {
    feedbackController.setPID(kP, kI, kD);
  }

  public boolean nearSpeedPoint() {
    return Math.abs(speedPoint - leftMotor.getVelocity().getValueAsDouble())
            < Constants.Shooter.SHOOTER_MAX
        && Math.abs(speedPoint - rightMotor.getVelocity().getValueAsDouble())
            < Constants.Shooter.SHOOTER_MAX;
  }
}
