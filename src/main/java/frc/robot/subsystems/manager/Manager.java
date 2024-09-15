package frc.robot.subsystems.manager;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ampBar.*;
import frc.robot.subsystems.drive.AutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavx2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOHybrid;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.NoteSimulator;

public class Manager extends Subsystem<ManagerStates> {

	Intake intakeSubsystem;
	AmpBar ampBarSubsystem;
	Shooter shooterSubsystem;
	Drive driveSubsystem;

	public Manager() {
		super("Manager", ManagerStates.IDLE);
		NoteSimulator.setDrive(driveSubsystem);

		switch (Constants.currentMode) {
			case REAL:
				intakeSubsystem = new Intake(new IntakeIOSparkMax());
				ampBarSubsystem = new AmpBar(new AmpBarIOReal());
				shooterSubsystem = new Shooter(new ShooterIOTalonFX());
				driveSubsystem = new Drive(
					new GyroIONavx2(SPI.Port.kMXP),
					new ModuleIOHybrid(0),
					new ModuleIOHybrid(1),
					new ModuleIOHybrid(2),
					new ModuleIOHybrid(3)
				);
				break;
			case REPLAY:
				intakeSubsystem = new Intake(new IntakeIO() {});
				ampBarSubsystem = new AmpBar(new AmpBarIO() {});
				shooterSubsystem = new Shooter(new ShooterIO() {});
				driveSubsystem = new Drive(
					new GyroIO() {},
					new ModuleIO() {},
					new ModuleIO() {},
					new ModuleIO() {},
					new ModuleIO() {}
				);
				break;
			case SIM:
				intakeSubsystem = new Intake(new IntakeIOSim());
				ampBarSubsystem = new AmpBar(new AmpBarIOSim());
				shooterSubsystem = new Shooter(new ShooterIOSim());
				driveSubsystem = new Drive(
					new GyroIO() {},
					new ModuleIOSim(),
					new ModuleIOSim(),
					new ModuleIOSim(),
					new ModuleIOSim()
				);
				break;
			default:
				break;
		}

		NoteSimulator.setDrive(driveSubsystem);
		AutoAlign.setDrive(driveSubsystem);

		// State Transitions (Nothing Automatic YET)

		/* Generally each action has a specific button, in intermediary states X will return to idle and you press
       the specific button to go through the states. Right now you can go through states without them being finished
       which should not be the case. Some state transitions that can be automated are also not automated. Most
    of the functions required for the TODOs are already build into the IOs!*/

		// TODO: Automatically return to idle after scoring amp (using a timer)
		// TODO: Automatically current sense notes to return to idle after intaking
		// TODO: Automatically go to the shooting state after spinning up if the shooting state is
		// entered from main controller input

		// Intaking (B)
		addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING, () ->
			Constants.controller.getBButtonPressed()
		);
		addTrigger(ManagerStates.INTAKING, ManagerStates.IDLE, () ->
			Constants.controller.getBButtonPressed()
		);
		addTrigger(ManagerStates.INTAKING, ManagerStates.IDLE, () -> intakeSubsystem.noteDetected());

		// Amping (Y)
		addTrigger(ManagerStates.IDLE, ManagerStates.FEED_AMP, () ->
			Constants.controller.getYButtonPressed()
		);
		addTrigger(ManagerStates.FEED_AMP, ManagerStates.SCORE_AMP, () ->
			Constants.controller.getYButtonPressed()
		);
		addTrigger(ManagerStates.FEED_AMP, ManagerStates.SCORE_AMP, () -> ampBarSubsystem.noteDetected());
		addTrigger(ManagerStates.SCORE_AMP, ManagerStates.IDLE, () ->
			Constants.controller.getYButtonPressed()
		);
		addTrigger(ManagerStates.SCORE_AMP, ManagerStates.IDLE, () -> 
			(getStateTime() > Constants.AmpBar.TIME_FOR_SCORING)
		);
		addTrigger(ManagerStates.SCORE_AMP, ManagerStates.IDLE, () ->
			Constants.controller.getYButtonPressed()
		);
		addTrigger(ManagerStates.FEED_AMP, ManagerStates.IDLE, () ->
			Constants.controller.getXButtonPressed()
		);

		// Shooting (A)
		addTrigger(ManagerStates.IDLE, ManagerStates.SPINNING_UP, () ->
			Constants.controller.getAButtonPressed()
		);
		addTrigger(ManagerStates.IDLE, ManagerStates.SPINNING_UP, () ->
			Constants.operatorController.getAButtonPressed()
		);
		addTrigger(ManagerStates.SPINNING_UP, ManagerStates.IDLE, () ->
			Constants.controller.getXButtonPressed()
		);
		addTrigger(ManagerStates.SPINNING_UP, ManagerStates.SHOOTING, () ->
			Constants.controller.getAButtonPressed()
		);
		addTrigger(ManagerStates.SHOOTING, ManagerStates.IDLE, () ->
			Constants.controller.getAButtonPressed()
		);
	}

	@Override
	public void periodic() {
		super.periodic();

		intakeSubsystem.setState(getState().getIntakeState());
		ampBarSubsystem.setState(getState().getAmpBarState());
		shooterSubsystem.setState(getState().getShooterState());
		shooterSubsystem.setManagerState(getState());

		intakeSubsystem.periodic();
		ampBarSubsystem.periodic();
		shooterSubsystem.periodic();
		driveSubsystem.periodic();

		// Cancel all actions regardless of whats happening
		if (Constants.operatorController.getXButtonPressed()) {
			setState(ManagerStates.IDLE);
		}
	}

	public void stop() {
		intakeSubsystem.stop();
		ampBarSubsystem.stop();
		shooterSubsystem.stop();
		driveSubsystem.stop();
	}

	@Override
	protected void runState() {
		NoteSimulator.update();
		NoteSimulator.logNoteInfo();
	}
}
