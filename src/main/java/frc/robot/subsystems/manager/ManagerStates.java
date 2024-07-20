package frc.robot.subsystems.manager;

import frc.robot.subsystems.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
    EXAMPLING("Exampling"),
    HALF_SPEEDING("Half Speeding"),
    IDLE("Idle");

	@Override
	public String getStateString() {
		return(this.stateString);
	}

    ManagerStates(String stateString) {
        this.stateString = stateString;
    }

    String stateString;
}