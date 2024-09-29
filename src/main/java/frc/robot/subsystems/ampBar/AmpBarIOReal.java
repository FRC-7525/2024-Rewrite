package frc.robot.subsystems.ampBar;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class AmpBarIOReal implements AmpBarIO {

	private final CANSparkMax leftMotor;
	// private final CANSparkMax rightMotor;
	private final TalonFX spinnerMotor;

	private RelativeEncoder pivotEncoder;
	private String stateString;
	private PIDController controller;

	private double pivotMotorAppliedVoltage;
	private double pivotPositionSetpoint;

	private double spinnerSpeedpoint;
	private double spinnerAppliedVoltage;

	private DigitalInput beamBreak;
	Debouncer beamBreakDebouncer;

	public AmpBarIOReal() {
		leftMotor = new CANSparkMax(Constants.AmpBar.LEFT_PIVOT_ID, MotorType.kBrushless);
		// rightMotor = new CANSparkMax(Constants.AmpBar.RIGHT_PIVOT_ID, MotorType.kBrushless);
		spinnerMotor = new TalonFX(Constants.AmpBar.SPINNER_ID);

		leftMotor.setInverted(false);
		// rightMotor.follow(leftMotor, true);
		leftMotor.setIdleMode(IdleMode.kCoast);
		// rightMotor.setIdleMode(IdleMode.kCoast);

		pivotEncoder = leftMotor.getEncoder();

		pivotEncoder.setPosition(0);
		pivotEncoder.setPositionConversionFactor(Constants.RADIAN_CF);
		pivotEncoder.setVelocityConversionFactor(Constants.RADIAN_CF);

		controller = new PIDController(0, 0, 0);

		stateString = "";
		pivotMotorAppliedVoltage = 0;
		pivotPositionSetpoint = 0;
		spinnerSpeedpoint = 0;
		spinnerAppliedVoltage = 0;

		beamBreak = new DigitalInput(Constants.AmpBar.BEAM_BREAK_PORT);
		beamBreakDebouncer = new Debouncer(
			Constants.AmpBar.DEBOUNCE_TIME,
			Debouncer.DebounceType.kBoth
		);
	}

	@Override
	public void updateInput(AmpBarIOInputs inputs) {
		inputs.ampBarState = stateString;

		inputs.pivotPosition = pivotEncoder.getPosition();
		inputs.pivotSetpoint = pivotPositionSetpoint;

		inputs.spinnerSpeed = getSpinnerSpeed();
		inputs.spinnerSetpoint = spinnerSpeedpoint;
	}

	@Override
	public void updateOutputs(AmpBarIOOutputs outputs) {
		outputs.pivotAppliedVoltage = pivotMotorAppliedVoltage;
		outputs.spinnerAppliedVoltage = spinnerAppliedVoltage;
	}

	@Override
	public void setPivotPosition(double position) {
		pivotPositionSetpoint = position;
		pivotMotorAppliedVoltage = controller.calculate(
			pivotEncoder.getPosition(),
			pivotPositionSetpoint
		);
		leftMotor.setVoltage(pivotMotorAppliedVoltage);
	}

	public void stop() {
		spinnerAppliedVoltage = 0;
		pivotMotorAppliedVoltage = 0;
		leftMotor.stopMotor();
		// rightMotor.stopMotor();
		spinnerMotor.stopMotor();
	}

	@Override
	public void setSpinnerSpeedpoint(double speed) {
		spinnerSpeedpoint = speed;
		spinnerAppliedVoltage = speed;
		spinnerMotor.setVoltage(speed);
	}

	@Override
	public double getPivotPosition() {
		return pivotEncoder.getPosition();
	}

	@Override
	public double getSpinnerSpeed() {
		return spinnerMotor.getVelocity().getValueAsDouble();
	}

	@Override
	public void configurePID(double kP, double kI, double kD) {
		controller.setPID(kP, kI, kD);
	}

	@Override
	public boolean nearSetPoint() {
		double motorPosition = getPivotPosition();
		return Math.abs(motorPosition - pivotPositionSetpoint) <= Constants.AmpBar.ERROR_OF_MARGIN;
	}

	@Override
	public boolean nearSpeedPoint() {
		return Math.abs(getSpinnerSpeed() - spinnerSpeedpoint) <= Constants.AmpBar.ERROR_OF_MARGIN;
	}

	@Override
	public boolean noteDetected() {
		return !beamBreakDebouncer.calculate(beamBreak.get());
	}
}
