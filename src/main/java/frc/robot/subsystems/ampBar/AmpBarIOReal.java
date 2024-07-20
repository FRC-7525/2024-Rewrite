package frc.robot.subsystems.ampBar;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;



public class AmpBarIOReal implements AmpBarIO {

    private final CANSparkMax leftMotor; 
    private final CANSparkMax rightMotor;
    private final TalonFX spinnerMotor;

    private RelativeEncoder pivotEncoder;
    private String stateString;
    private PIDController controller;
    
    private double pivotMotorAppliedVoltage;
    private double pivotPositionSetpoint;

    private double spinnerSpeedpoint;
    private double spinnerAppliedVoltage;

    final double ERROR_OF_MARGIN = 0.1;

    public AmpBarIOReal()
    {
        leftMotor = new CANSparkMax(31, MotorType.kBrushless);
        rightMotor = new CANSparkMax(30, MotorType.kBrushless);
        spinnerMotor = new TalonFX(38);

        pivotEncoder = leftMotor.getEncoder();
        stateString = "";
        controller = new PIDController(0, 0, 0);

        pivotMotorAppliedVoltage = 0;
        pivotPositionSetpoint = 0;

        spinnerSpeedpoint = 0;
        spinnerAppliedVoltage = 0;

        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor, true);
        pivotEncoder.setPosition(0);
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);

        pivotEncoder.setPositionConversionFactor(Math.PI * 2);
        pivotEncoder.setVelocityConversionFactor(Math.PI * 2);

    }

    @Override
    public void updateInput(AmpBarIOInputs inputs) {
        inputs.ampBarState = stateString;

        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotSetpoint = pivotPositionSetpoint;
        inputs.pivotAppliedVoltage = pivotMotorAppliedVoltage;

        inputs.spinnerSpeed = getSpinnerSpeed();
        inputs.spinnerSetpoint = spinnerSpeedpoint;
        inputs.spinnerAppliedVoltage = spinnerAppliedVoltage;
    }

    @Override
    public void setPivotPosition(double position) {
        pivotPositionSetpoint = position;
        pivotMotorAppliedVoltage = controller.calculate(pivotEncoder.getPosition(), pivotPositionSetpoint);
        leftMotor.setVoltage(pivotMotorAppliedVoltage);
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
    public boolean atSetPoint() {
        double motorPosition = getPivotPosition();
        return Math.abs(motorPosition - pivotPositionSetpoint) <= ERROR_OF_MARGIN;
    }
}
