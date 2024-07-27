package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  SingleJointedArmSim pivotSimModel;
  DCMotorSim spinnerWheelSim;

  PIDController outPivotController;
  PIDController inPIDController;

  PIDController pivotController;

  double wheelAppliedVoltage;
  double pivotAppliedVoltage;

  double wheelSpeedpoint;
  double pivotSetpoint;

  boolean usingInPID;

  public IntakeIOSim() {
    pivotSimModel =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Constants.Intake.PIVOT_GEARING, // gearing
            Constants.Intake.PIVOT_MOI, // MOI
            Constants.Intake.PIVOT_LENGTH_METERS, // arm length
            0, // min angle -- hard stop
            Constants.Intake.MAX_PIVOT_POSITION, // max angle -- floor
            false,
            0);

    spinnerWheelSim =
        new DCMotorSim(
            DCMotor.getFalcon500(1),
            Constants.Intake.SPINNER_GEARING,
            Constants.Intake.SPINNER_MOI);

    outPivotController = new PIDController(0, 0, 0);
    inPIDController = new PIDController(0, 0, 0);
  }

  public void setSetpoints(
      double pivotMotorSetPoint, double intakeMotorSetpoint, boolean useInPID) {
    usingInPID = useInPID;

    if (useInPID) pivotController = inPIDController;
    else pivotController = outPivotController;

    pivotAppliedVoltage =
        pivotController.calculate(pivotSimModel.getAngleRads(), pivotMotorSetPoint);

    wheelAppliedVoltage = intakeMotorSetpoint;

    pivotSimModel.setInputVoltage(pivotAppliedVoltage);
    spinnerWheelSim.setInputVoltage(intakeMotorSetpoint);

    wheelSpeedpoint = intakeMotorSetpoint;
    pivotSetpoint = pivotMotorSetPoint;
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.wheelSpeed = spinnerWheelSim.getAngularVelocityRPM() / 60;
    inputs.wheelAppliedVoltage = wheelAppliedVoltage;
    inputs.wheelSpeedpoint = wheelSpeedpoint;

    inputs.pivotPosition = pivotSimModel.getAngleRads();
    inputs.pivotAppliedVoltage = pivotAppliedVoltage;
    inputs.pivotSetpoint = pivotSetpoint;
    inputs.pivotSetpointError = Math.abs(pivotSimModel.getAngleRads() - pivotSetpoint);

    inputs.usingInPID = usingInPID;

    pivotSimModel.update(Constants.SIM_UPDATE_TIME);
    spinnerWheelSim.update(Constants.SIM_UPDATE_TIME);
  }

  public double getPosition() {
    return pivotSimModel.getAngleRads();
  }

  public void stop() {
    wheelAppliedVoltage = 0.0;
    pivotAppliedVoltage = 0.0;
    pivotSimModel.setInputVoltage(0);
    spinnerWheelSim.setInputVoltage(0);
  }

  public void configurePID(PIDConstants outPIDConst, PIDConstants inPIPidConst) {
    outPivotController.setPID(outPIDConst.kP, outPIDConst.kI, outPIDConst.kD);
    inPIDController.setPID(inPIPidConst.kP, inPIPidConst.kI, inPIPidConst.kD);
  }
}
