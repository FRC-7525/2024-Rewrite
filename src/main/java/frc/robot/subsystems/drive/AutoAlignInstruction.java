package frc.robot.subsystems.drive;

import frc.robot.subsystems.manager.ManagerStates;

public enum AutoAlignInstruction {
	SCORE_AMP_BAR(ManagerStates.FEED_AMP, ManagerStates.SCORE_AMP),
	SHOOT(ManagerStates.SPINNING_UP, ManagerStates.SHOOTING);

	AutoAlignInstruction(ManagerStates duringDrive, ManagerStates atSetPoint) {
		this.duringDrive = duringDrive;
		this.atSetPoint = atSetPoint;
	}

	ManagerStates duringDrive;
	ManagerStates atSetPoint;
}
