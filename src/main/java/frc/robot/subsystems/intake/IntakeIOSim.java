package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
        pivotSimModel = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            Constants.Intake.GEARING, // gearing
            0.192383865, // MOI
            0.3, // arm length
            Units.degreesToRadians(0), // min angle -- hard stop 
            Units.degreesToRadians(180), // max angle -- floor
            false, 
            Units.degreesToRadians(0)
        );

        spinnerWheelSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.Intake.GEARING, 0.192383865);

        outPivotController = new PIDController(0., 0, 0);
        inPIDController = new PIDController(0, 0, 0);
    }

    public void setSetpoints(double pivotMotorSetPoint, double intakeMotorSetpoint, boolean useInPID) {
        usingInPID = useInPID;

        if (useInPID) pivotController = inPIDController;
        else pivotController = outPivotController;

        pivotAppliedVoltage = pivotController.calculate(pivotSimModel.getVelocityRadPerSec() * 60, pivotMotorSetPoint);
        wheelAppliedVoltage = intakeMotorSetpoint;

        pivotSimModel.setInputVoltage(pivotAppliedVoltage);
        spinnerWheelSim.setInputVoltage(intakeMotorSetpoint);
        
        wheelSpeedpoint = intakeMotorSetpoint;
        pivotSetpoint = pivotMotorSetPoint;
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.wheelSpeed = spinnerWheelSim.getAngularVelocityRadPerSec() * 60;
        inputs.wheelAppliedVoltage = wheelAppliedVoltage;
        inputs.wheelSpeedpoint = wheelSpeedpoint;

        inputs.pivotPosition = pivotSimModel.getAngleRads();
        inputs.pivotAppliedVoltage = pivotAppliedVoltage;
        inputs.pivotSetpoint = pivotSetpoint;
        inputs.pivotSetpointError = Math.abs(pivotSimModel.getAngleRads() - pivotSetpoint);

        inputs.usingInPID = usingInPID;
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

    public void configurePID(PIDController outPivotController, PIDController inPIPidController) {
        this.outPivotController = outPivotController;
        this.inPIDController = inPIPidController;
    }
}
