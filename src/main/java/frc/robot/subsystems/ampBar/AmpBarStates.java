package frc.robot.subsystems.ampBar;

import frc.robot.subsystems.SubsystemStates;

public enum AmpBarStates implements SubsystemStates {
  OFF(0, 0, "Amp Bar Off"),
  SHOOTING(-0.717, -0.5, "Shooting Amp"),
  IN(0, 0, "Amp Bar In"),
  FEEDING(-0.625, -0.1, "Getting Fed"),
  HOLDING_NOTE(-0.717, 0, "Holding a Note");

  private double pivotPositionSetpoint;
  private double spinnerMotorSpeedpoint;
  private String stateString;

  AmpBarStates(double pivotMotorSetpoint, double spinnerMotorSpeedpoint, String stateString) {
    this.pivotPositionSetpoint = pivotMotorSetpoint;
    this.spinnerMotorSpeedpoint = spinnerMotorSpeedpoint;
    this.stateString = stateString;
  }

  public double getPivotPositionSetpoint() {
    return pivotPositionSetpoint;
  }

  public double getMotorSpeedpoint() {
    return spinnerMotorSpeedpoint;
  }

  @Override
  public String getStateString() {
    return stateString;
  }
}
