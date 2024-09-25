package frc.robot.subsystems.manager;

import frc.robot.subsystems.SubsystemStates;
import frc.robot.subsystems.ampBar.AmpBarStates;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.shooter.ShooterStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("Idle", IntakeStates.OFF, AmpBarStates.OFF, ShooterStates.OFF),
	INTAKING("Intaking", IntakeStates.INTAKING, AmpBarStates.OFF, ShooterStates.OFF),
	SPINNING_UP("Spinning Up", IntakeStates.OFF, AmpBarStates.OFF, ShooterStates.SHOOTING),
	SPINNING_AND_INTAKING(
		"Spin and Intake",
		IntakeStates.INTAKING,
		AmpBarStates.OFF,
		ShooterStates.SHOOTING
	),
	SHOOTING("Shooting", IntakeStates.FEEDING, AmpBarStates.OFF, ShooterStates.SHOOTING),
	FEED_AMP("Feed Amp", IntakeStates.FEEDING, AmpBarStates.FEEDING, ShooterStates.PASSING_AMP),
	AMP_HOLDING_NOTE("Holding Note", IntakeStates.OFF, AmpBarStates.HOLDING_NOTE, ShooterStates.OFF),
	SCORE_AMP("Score Amp", IntakeStates.OFF, AmpBarStates.SHOOTING, ShooterStates.OFF);

	String stateString;
	IntakeStates intakeState;
	AmpBarStates ampBarState;
	ShooterStates shooterState;

	ManagerStates(
		String stateString,
		IntakeStates intakeState,
		AmpBarStates ampBarState,
		ShooterStates shooterState
	) {
		this.stateString = stateString;
		this.intakeState = intakeState;
		this.ampBarState = ampBarState;
		this.shooterState = shooterState;
	}

	public String getStateString() {
		return this.stateString;
	}

	public IntakeStates getIntakeState() {
		return intakeState;
	}

	public AmpBarStates getAmpBarState() {
		return ampBarState;
	}

	public ShooterStates getShooterState() {
		return shooterState;
	}
}
