package frc.robot.subsystems.AutoAlign;

import frc.robot.subsystems.SubsystemStates;

public enum AutoAlignStates implements SubsystemStates {
  ON("ON"),
  OFF("OFF");

  private String stateString;

  AutoAlignStates(String stateString) {
    this.stateString = stateString;
  }

  @Override
  public String getStateString() {
    return stateString;
  }
}
