package frc.robot.subsystems.ampBar;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AmpBarIOSim implements AmpBarIO {
    SingleJointedArmSim pivotSim;
    DCMotorSim spinnerSim;
    PIDController controller;

    String stateString;

    double pivotAppliedVoltage;
    double pivotSetpoint;
    
    double spinnerAppliedVoltage;
    double spinnerSpeedpoint;

    final double ERROR_OF_MARGIN = .1;

    public AmpBarIOSim() {
        //the sim values are random
        pivotSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            .5,
            2,
            1,
            Math.PI,
            Math.PI,
            false,
            0
        );
        spinnerSim = new DCMotorSim(DCMotor.getKrakenX60(0), .5, .5);

        controller = new PIDController(0, 0, 0);
        stateString = "";

        pivotAppliedVoltage = 0;
        pivotSetpoint = 0;
        spinnerAppliedVoltage =0;
        spinnerSpeedpoint=0;
    }   

    @Override
    public void updateInput(AmpBarIOInputs inputs) {
        inputs.mechanismPose3d = new Pose3d(
            new Translation3d(0,0,0),
            new Rotation3d(0, getPivotPosition(), 0)
        );
        inputs.ampBarState = stateString;

        inputs.pivotPosition = getPivotPosition();
        inputs.pivotSetpoint = pivotSetpoint;
        inputs.pivotAppliedVoltage = pivotAppliedVoltage;

        inputs.spinnerSpeed = getSpinnerSpeed();
        inputs.spinnerSetpoint = spinnerSpeedpoint;
        inputs.spinnerAppliedVoltage = spinnerAppliedVoltage;
    }

    @Override
    public void setPivotPosition(double position) {
        pivotSetpoint = position;
        pivotAppliedVoltage = controller.calculate(pivotSim.getVelocityRadPerSec() * 60, pivotSetpoint);
        pivotSim.setInputVoltage(pivotAppliedVoltage);
    }

    @Override
    public void setSpinnerSpeedpoint(double speed) {
        spinnerSpeedpoint = speed;
        spinnerAppliedVoltage = speed;
        spinnerSim.setInputVoltage(speed);
    }

    @Override
    public double getPivotPosition() {
        return pivotSim.getAngleRads();
    }

    @Override
    public double getSpinnerSpeed() {
        return spinnerSim.getAngularVelocityRadPerSec() * 60;
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    @Override
    public void setStateString(String inputStateString) {
        stateString = inputStateString;
    }

    @Override
    public boolean atSetPoint() {
        double motorPosition = getPivotPosition();
        return Math.abs(motorPosition - pivotSetpoint) <= ERROR_OF_MARGIN;
    }
    
}
