package frc.robot.subsystems.manager;

import frc.robot.subsystems.SubsystemStates;
import frc.robot.subsystems.ampBar.AmpBarStates;
import frc.robot.subsystems.climbers.ClimberStates;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.shooter.ShooterStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("Idle", IntakeStates.OFF, AmpBarStates.OFF, ShooterStates.OFF, ClimberStates.OFF),
	INTAKING("Intaking", IntakeStates.INTAKING, AmpBarStates.OFF, ShooterStates.OFF, ClimberStates.OFF),
	SPINNING_UP("Spinning Up", IntakeStates.OFF, AmpBarStates.OFF, ShooterStates.SHOOTING, ClimberStates.OFF),
	SPINNING_AND_INTAKING(
		"Spin and Intake",
		IntakeStates.INTAKING,
		AmpBarStates.OFF,
		ShooterStates.SHOOTING,
		ClimberStates.OFF
	),
	SHOOTING("Shooting", IntakeStates.FEEDING, AmpBarStates.OFF, ShooterStates.SHOOTING, ClimberStates.OFF),
	FEED_AMP("Feed Amp", IntakeStates.FEEDING, AmpBarStates.FEEDING, ShooterStates.PASSING_AMP, ClimberStates.OFF),
	SCORE_AMP("Score Amp", IntakeStates.OFF, AmpBarStates.SHOOTING, ShooterStates.OFF, ClimberStates.OFF),
	STAGING_FOR_CLIMG(
		"Staging for Climbing",
		IntakeStates.OFF,
		AmpBarStates.HOLDING_NOTE,
		ShooterStates.OFF,
		ClimberStates.OFF
	),
	CLIMBING("Climbing", IntakeStates.OFF, AmpBarStates.FEEDING, ShooterStates.OFF, ClimberStates.CLIMBING);

	String stateString;
	IntakeStates intakeState;
	AmpBarStates ampBarState;
	ShooterStates shooterState;
	ClimberStates climberState;

	ManagerStates(
		String stateString,
		IntakeStates intakeState,
		AmpBarStates ampBarState,
		ShooterStates shooterState,
		ClimberStates climberState
	) {
		this.stateString = stateString;
		this.intakeState = intakeState;
		this.ampBarState = ampBarState;
		this.shooterState = shooterState;
		this.climberState = climberState;
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

	public ClimberStates getClimberState() {
		return climberState;
	}
}
