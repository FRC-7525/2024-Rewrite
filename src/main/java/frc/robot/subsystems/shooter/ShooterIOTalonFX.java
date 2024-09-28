package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

	private TalonFX leftMotor;
	private TalonFX rightMotor;
	private BangBangController feedbackController;
	private double speedPoint;
	private double leftAppliedVolts;
	private double rightAppliedVolts;
	private StatusSignal<Double> leftVelocity;
	private StatusSignal<Double> rightVelocity;
	private StatusSignal<Double> leftAmps;
	private StatusSignal<Double> rightAmps;

	PIDController shooterPIDController;

	public ShooterIOTalonFX() {
		feedbackController = new BangBangController();
		leftMotor = new TalonFX(Constants.Shooter.LEFT_SHOOTER_ID);
		leftVelocity = leftMotor.getVelocity();
		leftAmps = leftMotor.getSupplyCurrent();

		rightMotor = new TalonFX(Constants.Shooter.RIGHT_SHOOTER_ID);
		rightVelocity = rightMotor.getVelocity();
		rightAmps = rightMotor.getSupplyCurrent();

		speedPoint = 0.0;
		leftMotor.setInverted(true);
		rightMotor.setInverted(false);
		BaseStatusSignal.setUpdateFrequencyForAll(
			Constants.SLOW_UPDATE_FREQ,
			rightVelocity,
			leftVelocity,
			rightAmps,
			leftAmps
		);
	}

	public void updateInputs(ShooterIOInputs inputs) {
		inputs.leftShooterSpeed = leftVelocity.getValueAsDouble();
		inputs.rightShooterSpeed = rightVelocity.getValueAsDouble();
		inputs.shooterSpeedPoint = speedPoint;
		inputs.rightShooterAmp = rightAmps.getValueAsDouble();
		inputs.leftShooterAmp = leftAmps.getValueAsDouble();
	}

	public void updateOutputs(ShooterIOOutputs outputs) {
		outputs.leftShooterAppliedVolts = leftAppliedVolts;
		outputs.rightShooterAppliedVolts = rightAppliedVolts;
	}

	public void setSpeed(double rps) {
		if (rps == 0) return;

		speedPoint = rps;
		leftAppliedVolts = feedbackController.calculate(leftVelocity.getValueAsDouble(), rps);

		// leftAppliedVolts = shooterPIDController.calculate(leftVelocity.getValueAsDouble(), rps);
		// rightAppliedVolts = shooterPIDController.calculate(rightVelocity.getValueAsDouble(), rps);

		rightAppliedVolts = feedbackController.calculate(rightVelocity.getValueAsDouble(), rps);

		leftMotor.set(leftAppliedVolts);
		rightMotor.set(rightAppliedVolts);
	}

	public void stop() {
		leftAppliedVolts = 0.0;
		rightAppliedVolts = 0.0;
		leftMotor.stopMotor();
		rightMotor.stopMotor();
	}

	public boolean nearSpeedPoint() {
		return (
			Math.abs(speedPoint - leftVelocity.getValueAsDouble()) <
				Constants.Shooter.ERROR_OF_MARGIN &&
			Math.abs(speedPoint - rightVelocity.getValueAsDouble()) <
			Constants.Shooter.ERROR_OF_MARGIN
		);
	}

	@Override
	public double getAverageSpeed() {
		return (
			(rightVelocity.getValueAsDouble() / Constants.AVG_TWO_ITEM_F) //+ rightMotor.getVelocity().getValueAsDouble())
		);
	}
}
