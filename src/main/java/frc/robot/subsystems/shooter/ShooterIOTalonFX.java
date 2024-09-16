package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

	private TalonFX leftMotor;
	// private TalonFX rightMotor;
	private BangBangController feedbackController;
	private double speedPoint;
	private double leftAppliedVolts;
	private double rightAppliedVolts;

	public ShooterIOTalonFX() {
		feedbackController = new BangBangController();
		leftMotor = new TalonFX(Constants.Shooter.LEFT_SHOOTER_ID);
		// rightMotor = new TalonFX(Constants.Shooter.RIGHT_SHOOTER_ID);
		speedPoint = 0.0;
		leftMotor.setInverted(false);
		// rightMotor.setInverted(true);
	}

	public void updateInputs(ShooterIOInputs inputs) {
		inputs.leftShooterSpeed = leftMotor.getVelocity().getValueAsDouble();
		// inputs.rightShooterSpeed = rightMotor.getVelocity().getValueAsDouble();
		inputs.shooterSpeedPoint = speedPoint;
	}

	public void updateOutputs(ShooterIOOutputs outputs) {
		outputs.leftShooterAppliedVolts = leftAppliedVolts;
		outputs.rightShooterAppliedVolts = rightAppliedVolts;
	}

	public void setSpeed(double rps) {
		speedPoint = rps;
		leftAppliedVolts = 12 *
		feedbackController.calculate(leftMotor.getRotorVelocity().getValueAsDouble(), rps);
		// rightAppliedVolts = feedbackController.calculate(
		//	rightMotor.getRotorVelocity().getValueAsDouble(),
		//	rps
		// );

		leftMotor.setVoltage(leftAppliedVolts);
		// rightMotor.setVoltage(rightAppliedVolts);
	}

	public void stop() {
		leftAppliedVolts = 0.0;
		rightAppliedVolts = 0.0;
		leftMotor.stopMotor();
		// rightMotor.stopMotor();
	}

	public boolean nearSpeedPoint() {
		return (
			Math.abs(speedPoint - leftMotor.getVelocity().getValueAsDouble()) <
			Constants.Shooter.ERROR_OF_MARGIN
			// Math.abs(speedPoint - rightMotor.getVelocity().getValueAsDouble()) <
			// Constants.Shooter.ERROR_OF_MARGIN
		);
	}

	@Override
	public double getAverageSpeed() {
		return (
			(leftMotor.getVelocity().getValueAsDouble() / Constants.AVG_TWO_ITEM_F) //+ rightMotor.getVelocity().getValueAsDouble())
		);
	}
}
