package frc.robot.subsystems.shooter;

import frc.robot.subsystems.SubsystemStates;

public enum ShooterStates implements SubsystemStates {
  OFF(0, "Shooter Off"),
  SHOOTING(1, "Shooting Full"),
  PASSING_AMP(0.5, "Passing To Amp");

  private double speedPoint;
  private String stateString;

  ShooterStates(double speedPoint, String stateString) {
    this.speedPoint = speedPoint;
    this.stateString = stateString;
  }

  public double getMotorSpeedpoint() {
    return speedPoint;
  }

  @Override
  public String getStateString() {
    return stateString;
  }
}
