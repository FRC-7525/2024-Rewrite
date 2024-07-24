package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

  double wheelAppliedVoltage;
  double pivotAppliedVoltage;

  double wheelSpeedpoint;
  double pivotSetpoint;

  boolean usingInPID;

  public IntakeIOSparkMax() {
    intakeMotor = new TalonFX(20);
    pivotMotor = new CANSparkMax(32, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getEncoder();

    outPivotController = new PIDController(0, 0, 0);
    inPIDController = new PIDController(0, 0, 0);

    pivotEncoder.setPositionConversionFactor(Math.PI * 2);
    pivotEncoder.setVelocityConversionFactor(Math.PI * 2);
  }

  public void setSetpoints(
      double pivotMotorSetpoint, double intakeMotorSetpoint, boolean useInPID) {
    usingInPID = useInPID;

    if (useInPID) pivotController = inPIDController;
    else pivotController = outPivotController;

    pivotAppliedVoltage = pivotController.calculate(pivotEncoder.getPosition(), pivotMotorSetpoint);
    wheelAppliedVoltage = intakeMotorSetpoint;

    pivotMotor.setVoltage(pivotAppliedVoltage);
    intakeMotor.setVoltage(wheelAppliedVoltage);

    wheelSpeedpoint = intakeMotorSetpoint;
    pivotSetpoint = pivotMotorSetpoint;
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.wheelSpeed = intakeMotor.getVelocity().getValueAsDouble();
    inputs.wheelAppliedVoltage = wheelAppliedVoltage;
    inputs.wheelSpeedpoint = wheelSpeedpoint;

    inputs.pivotPosition = pivotEncoder.getPosition();
    inputs.pivotAppliedVoltage = pivotAppliedVoltage;
    inputs.pivotSetpoint = pivotSetpoint;
    inputs.pivotSetpointError = Math.abs(pivotEncoder.getPosition() - pivotSetpoint);

    inputs.usingInPID = usingInPID;
  }

  public double getPosition() {
    return pivotEncoder.getPosition();
  }

  public void stop() {
    wheelAppliedVoltage = 0.0;
    pivotAppliedVoltage = 0.0;
    pivotMotor.stopMotor();
    intakeMotor.stopMotor();
  }

  public void configurePID(PIDConstants outPIDConst, PIDConstants inPIPidConst) {
    outPivotController.setPID(outPIDConst.kP, outPIDConst.kI, outPIDConst.kD);
    inPIDController.setPID(inPIPidConst.kP, inPIPidConst.kI, inPIPidConst.kD);
  }
}
