package frc.robot.subsystems.shooter;

import frc.robot.subsystems.SubsystemStates;

public enum ShooterStates implements SubsystemStates {
  OFF(0, "Shooter Off"),
  SHOOTING(1, "Shooting Full"),
  PASSING_AMP(0.5, "Passing To Amp");

  private double spinnerMotorSpeedpoint;
  private String stateString;

  ShooterStates(double spinnerMotorSpeedpoint, String stateString) {
    this.spinnerMotorSpeedpoint = spinnerMotorSpeedpoint;
    this.stateString = stateString;
  }

  public double getMotorSpeedpoint() {
    return spinnerMotorSpeedpoint;
  }

  @Override
  public String getStateString() {
    return stateString;
  }
}
