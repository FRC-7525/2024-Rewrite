package frc.robot.subsystems.ampBar;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
    // the sim values are random
    pivotSim =
    new SingleJointedArmSim(DCMotor.getNEO(2), .5, 2, .378, Units.degreesToRadians(179), Units.degreesToRadians(293.1633329) * 2, false, Units.degreesToRadians(179));
    //still need gearing, and jkgMetersSquared
    spinnerSim = new DCMotorSim(DCMotor.getKrakenX60(1), .5, .5);

    controller = new PIDController(0, 0, 0);
    stateString = "";

    pivotAppliedVoltage = 0;
    pivotSetpoint = 0;
    spinnerAppliedVoltage = 0;
    spinnerSpeedpoint = 0;
  }

  @Override
  public void updateInput(AmpBarIOInputs inputs) {
    inputs.ampBarState = stateString;

    inputs.pivotPosition = getPivotPosition();
    inputs.pivotSetpoint = pivotSetpoint;
    inputs.pivotAppliedVoltage = pivotAppliedVoltage;

    inputs.spinnerSpeed = getSpinnerSpeed();
    inputs.spinnerSetpoint = spinnerSpeedpoint;
    inputs.spinnerAppliedVoltage = spinnerAppliedVoltage;

    pivotSim.update(0.05);
    spinnerSim.update(0.05);
  }

  @Override
  public void setPivotPosition(double position) {
    pivotSetpoint = position;
    pivotAppliedVoltage = controller.calculate(pivotSim.getAngleRads(), pivotSetpoint);
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
    return spinnerSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }

  @Override
  public boolean atSetPoint() {
    double motorPosition = getPivotPosition();
    return Math.abs(motorPosition - pivotSetpoint) <= ERROR_OF_MARGIN;
  }
}
