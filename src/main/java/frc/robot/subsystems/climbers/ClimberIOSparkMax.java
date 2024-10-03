package frc.robot.subsystems.climbers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;

public class ClimberIOSparkMax implements ClimberIO {

	private CANSparkMax leftClimberMotor;
	private CANSparkMax rightClimberMotor;

	private RelativeEncoder leftClimberEncoder;
	private RelativeEncoder rightClimberEncoder;

	private PIDController climberController;

	private LinearFilter leftFilter;
	private LinearFilter rightFilter;

	private boolean leftClimberZeroed;
	private boolean rightClimberZeroed;

	private double leftClimberSetpoint;
	private double rightClimberSetpoint;

	private double leftAppliedVolts;
	private double rightAppliedVolts;

	public ClimberIOSparkMax() {
		leftClimberMotor = new CANSparkMax(
			Constants.Climber.LEFT_ID,
			CANSparkMax.MotorType.kBrushless
		);
		rightClimberMotor = new CANSparkMax(
			Constants.Climber.RIGHT_ID,
			CANSparkMax.MotorType.kBrushless
		);

		leftClimberMotor.setInverted(false);
		rightClimberMotor.setInverted(true);

		leftClimberEncoder = leftClimberMotor.getEncoder();
		rightClimberEncoder = rightClimberMotor.getEncoder();

		climberController = new PIDController(0, 0, 0);

		leftFilter = LinearFilter.movingAverage(5);
		rightFilter = LinearFilter.movingAverage(5);

		leftClimberSetpoint = 0;
		rightClimberSetpoint = 0;

		leftClimberZeroed = false;
		rightClimberZeroed = false;

		leftAppliedVolts = 0;
		rightAppliedVolts = 0;
	}

	public void updateInputs(ClimberIOInputs inputs) {
		inputs.leftClimberPosition = leftClimberEncoder.getPosition();
		inputs.rightClimberPosition = rightClimberEncoder.getPosition();
		inputs.leftClimberSetpoint = leftClimberSetpoint;
		inputs.rightClimberSetpoint = rightClimberSetpoint;
		inputs.leftClimberSpeed = leftClimberEncoder.getVelocity() / 60;
		inputs.rightClimberSpeed = rightClimberEncoder.getVelocity() / 60;
	}

	public void updateOutputs(ClimberIOOutputs outputs) {
		outputs.leftClimberAppliedVoltage = leftAppliedVolts;
		outputs.rightClimberAppliedVoltage = rightAppliedVolts;
	}

	public boolean nearSetpoints() {
		return (
			Math.abs(leftClimberEncoder.getPosition() - leftClimberEncoder.getPosition()) <
				Constants.Climber.ERROR_OF_MARGIN &&
			Math.abs(rightClimberEncoder.getPosition() - rightClimberEncoder.getPosition()) <
			Constants.Climber.ERROR_OF_MARGIN
		);
	}

	public boolean climbersZeroed() {
		return leftClimberZeroed && rightClimberZeroed;
	}

	public void configurePID(double kP, double kI, double kD) {
		climberController.setPID(kP, kI, kD);
	}

	public void setSetpoints(double leftSetpoint, double rightSetpoint) {
		this.leftClimberSetpoint = leftSetpoint;
		this.rightClimberSetpoint = rightSetpoint;

		this.leftAppliedVolts = climberController.calculate(
			leftClimberEncoder.getPosition(),
			leftSetpoint
		);
		this.rightAppliedVolts = climberController.calculate(
			rightClimberEncoder.getPosition(),
			rightSetpoint
		);

		leftClimberMotor.set(leftAppliedVolts);
		rightClimberMotor.set(rightAppliedVolts);
	}

	public void zeroClimbers() {
		double leftZeroingSpeed = -0.25;
		double rightZeroingSpeed = -0.25;

		// Not voltage sensing
		if (
			leftFilter.calculate(leftClimberMotor.getOutputCurrent()) >
				Constants.Climber.LEFT_CURRENT_LIMIT ||
			leftClimberZeroed
		) {
			if (!leftClimberZeroed) leftClimberMotor.getEncoder().setPosition(0);
			leftClimberSetpoint = 10;
			leftZeroingSpeed = climberController.calculate(
				leftClimberEncoder.getPosition(),
				leftClimberSetpoint
			);
			leftClimberZeroed = true;
		}

		if (
			rightFilter.calculate(rightClimberMotor.getOutputCurrent()) >
				Constants.Climber.RIGHT_CURRENT_LIMIT ||
			rightClimberZeroed
		) {
			if (!rightClimberZeroed) rightClimberMotor.getEncoder().setPosition(0);
			rightClimberSetpoint = 10;
			rightZeroingSpeed = climberController.calculate(
				rightClimberEncoder.getPosition(),
				rightClimberSetpoint
			);
			rightClimberZeroed = true;
		}

		leftClimberMotor.set(leftZeroingSpeed);
		rightClimberMotor.set(rightZeroingSpeed);
	}

	public void stop() {
		leftClimberMotor.stopMotor();
		rightClimberMotor.stopMotor();
	}
}
