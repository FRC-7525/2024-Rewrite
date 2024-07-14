package frc.robot.subsystems.exampleSubsystem;

import frc.robot.subsystems.SubsystemStates;

public enum ExampleStates implements SubsystemStates {
  ON("Wheel On", 1.0),
  HALF_SPEED("Wheel Half Speed", 0.5),
  OFF("Wheel Off", 0.0);

  private double speedPoint;
  private String stateString;

  ExampleStates(String stateString, double speedPoint) {
    this.speedPoint = speedPoint;
    this.stateString = stateString;
  }

  public String getStateString() {
    return stateString;
  }

  public double getSpeedPoint() {
    return speedPoint;
  }
}
