package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeIO;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.math.controller.PIDController;

public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax pivotMotor;
    public TalonFX intakeMotor;
    private RelativeEncoder pivotEncoder;

    PIDController outPivotController;
    PIDController inPIDController;

    PIDController pivotController;

    double pivotMotorSetpoint = 0.0;
    double intakeMotorSetpoint = 0.0;

    private IntakeStates currentState;

    double wheelAppliedVoltage;
    double pivotAppliedVoltage;

    double wheelSpeedpoint;
    double pivotSetpoint;

    public IntakeIOSparkMax() {
        intakeMotor = new TalonFX(20);
        pivotMotor = new CANSparkMax(32, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        outPivotController = new PIDController(0.09, 0, 0);
        inPIDController = new PIDController(0.05, 0, 0);

        pivotEncoder.setPositionConversionFactor(Math.PI * 2);
        pivotEncoder.setVelocityConversionFactor(Math.PI * 2);
    }

    public void setSetpoints(double pivotMotorSetpoint, double intakeMotorSetpoint, boolean useIn) {
        if (useIn) pivotController = inPIDController;
        else pivotController = outPivotController;

        pivotAppliedVoltage = pivotController.calculate(pivotEncoder.getPosition(), pivotMotorSetpoint);
        wheelAppliedVoltage = intakeMotorSetpoint;

        pivotMotor.setVoltage(pivotAppliedVoltage);
        intakeMotor.setVoltage(wheelAppliedVoltage);

        wheelSpeedpoint = intakeMotorSetpoint;
        pivotSetpoint = pivotMotorSetpoint;
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.stateString = currentState.getStateString();
        inputs.position = new Pose3d(
            new Translation3d(0,0,0),
            new Rotation3d(0, pivotEncoder.getPosition(), 0)
        );

        inputs.wheelSpeed = intakeMotor.getVelocity().getValueAsDouble();
        inputs.wheelAppliedVoltage = wheelAppliedVoltage;
        inputs.wheelSpeedpoint = wheelSpeedpoint;

        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotAppliedVoltage = pivotAppliedVoltage;
        inputs.pivotSetpoint = pivotSetpoint;
        inputs.pivotSetpointError = Math.abs(pivotEncoder.getPosition() - pivotSetpoint);
    }

    public double getPosition() {
        return pivotEncoder.getPosition();
    }

    public void setState(IntakeStates state) {
        currentState = state;
    }

    public void stop() {
        wheelAppliedVoltage = 0.0;
        pivotAppliedVoltage = 0.0;
        pivotMotor.stopMotor();
        intakeMotor.stopMotor();
    }

    public void configurePID(PIDController outPivotController, PIDController inPIPidController) {
        this.outPivotController = outPivotController;
        this.inPIDController = inPIPidController;
    }
}
