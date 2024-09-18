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
	private StatusSignal leftVelocity;
	private StatusSignal rightVelocity;
	PIDController shooterPIDController;
	

	public ShooterIOTalonFX() {

		shooterPIDController = new PIDController(1, 0, 0);

		feedbackController = new BangBangController();
		leftMotor = new TalonFX(Constants.Shooter.LEFT_SHOOTER_ID);
		leftVelocity = leftMotor.getVelocity();
		// leftMotor.getVelocity().setUpdateFrequency(50);

		rightMotor = new TalonFX(Constants.Shooter.RIGHT_SHOOTER_ID);
		rightVelocity = rightMotor.getVelocity();
		speedPoint = 0.0;
		leftMotor.setInverted(true);
		rightMotor.setInverted(false);
		// rightMotor.getVelocity().setUpdateFrequency(50);
		BaseStatusSignal.setUpdateFrequencyForAll(50, rightVelocity, leftVelocity);
	}

	public void updateInputs(ShooterIOInputs inputs) {
		inputs.leftShooterSpeed = leftMotor.getVelocity().getValueAsDouble();
		inputs.rightShooterSpeed = rightMotor.getVelocity().getValueAsDouble();
		inputs.shooterSpeedPoint = speedPoint;
		inputs.rightShooterAmp = rightMotor.getSupplyCurrent().getValueAsDouble();
		inputs.leftShooterAmp = leftMotor.getSupplyCurrent().getValueAsDouble();
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

		rightAppliedVolts = feedbackController.calculate(
			rightVelocity.getValueAsDouble(),
			rps
		);

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
			// Math.abs(speedPoint - leftVelocity.getValueAsDouble()) <
			// Constants.Shooter.ERROR_OF_MARGIN
			Math.abs(speedPoint - rightMotor.getVelocity().getValueAsDouble()) <
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
