package frc.robot.subsystems.manager;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.AutoAlign.AutoAlign;
import frc.robot.subsystems.AutoAlign.AutoAlignIO;
import frc.robot.subsystems.ampBar.*;
import frc.robot.subsystems.climbers.Climber;
import frc.robot.subsystems.climbers.ClimberIO;
import frc.robot.subsystems.climbers.ClimberIOSim;
import frc.robot.subsystems.climbers.ClimberIOSparkMax;
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

	private Climber climberSubsystem;
	private Intake intakeSubsystem;
	private AmpBar ampBarSubsystem;
	private Shooter shooterSubsystem;
	private Drive driveSubsystem;
	private AutoAlign autoAlignSubsystem;

	private SendableChooser<Boolean> useBeamBreaks;
	private SendableChooser<Boolean> useAutoAlign;
	private SendableChooser<Boolean> driverShooterAfterSpinning;

	public Manager() {
		super("Manager", ManagerStates.IDLE);
		NoteSimulator.setDrive(driveSubsystem);

		useBeamBreaks = new SendableChooser<>();
		useAutoAlign = new SendableChooser<>();
		driverShooterAfterSpinning = new SendableChooser<>();

		useBeamBreaks.setDefaultOption("On", true);
		useBeamBreaks.addOption("Off", false);

		useAutoAlign.setDefaultOption("On", true);
		useAutoAlign.addOption("Off", false);

		driverShooterAfterSpinning.setDefaultOption("On", true);
		driverShooterAfterSpinning.addOption("Off", false);

		SmartDashboard.putData("Beam Breaks Toggle", useBeamBreaks);
		SmartDashboard.putData("Auto Align Toggle", useAutoAlign);
		SmartDashboard.putData("Drive Shoot After Spinning Toggle", driverShooterAfterSpinning);

		switch (Constants.currentMode) {
			case REAL:
				intakeSubsystem = new Intake(new IntakeIOSparkMax());
				ampBarSubsystem = new AmpBar(new AmpBarIOReal());
				shooterSubsystem = new Shooter(new ShooterIOTalonFX());
				climberSubsystem = new Climber(new ClimberIOSparkMax());
				driveSubsystem = new Drive(
					new GyroIONavx2(),
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
				climberSubsystem = new Climber(new ClimberIO() {});
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
				climberSubsystem = new Climber(new ClimberIOSim());
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
		autoAlignSubsystem = new AutoAlign(new AutoAlignIO(driveSubsystem));

		NoteSimulator.setDrive(driveSubsystem);

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
		addTrigger(
			ManagerStates.INTAKING,
			ManagerStates.IDLE,
			() ->
				intakeSubsystem.noteDetected() &&
				intakeSubsystem.nearSetpoints() &&
				useBeamBreaks.getSelected()
		);

		// Amping (Y)
		addTrigger(ManagerStates.IDLE, ManagerStates.STAGING_AMP, () ->
			Constants.controller.getYButtonPressed()
		);
		addTrigger(ManagerStates.STAGING_AMP, ManagerStates.FEED_AMP, () ->
			ampBarSubsystem.atSetPoint()
		);
		addTrigger(
			ManagerStates.FEED_AMP,
			ManagerStates.AMP_HOLDING_NOTE,
			() ->
				(ampBarSubsystem.noteDetected() &&
					(useBeamBreaks.getSelected() == null ? true : useBeamBreaks.getSelected())) ||
				Constants.controller.getYButtonPressed()
		);
		addTrigger(ManagerStates.AMP_HOLDING_NOTE, ManagerStates.SCORE_AMP, () ->
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
		addTrigger(ManagerStates.IDLE, ManagerStates.OPERATOR_SPINNING_UP, () ->
			Constants.operatorController.getAButtonPressed()
		);
		addTrigger(
			ManagerStates.SPINNING_UP,
			ManagerStates.SHOOTING,
			() ->
				((driverShooterAfterSpinning.getSelected() == null
							? true
							: driverShooterAfterSpinning.getSelected()) &&
					shooterSubsystem.nearSpeedPoint()) ||
				Constants.controller.getAButtonPressed()
		);
		addTrigger(ManagerStates.SPINNING_UP, ManagerStates.IDLE, () ->
			Constants.controller.getXButtonPressed()
		);
		addTrigger(ManagerStates.OPERATOR_SPINNING_UP, ManagerStates.SHOOTING, () ->
			Constants.controller.getAButtonPressed()
		);
		addTrigger(ManagerStates.SHOOTING, ManagerStates.IDLE, () ->
			Constants.controller.getAButtonPressed()
		);

		// Climbing
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.STAGING_FOR_CLIMB,
			() -> Constants.controller.getPOV() == 0
		);
		addTrigger(ManagerStates.STAGING_FOR_CLIMB, ManagerStates.CLIMBING, () ->
			ampBarSubsystem.nearSetPoints()
		);
		addTrigger(
			ManagerStates.CLIMBING,
			ManagerStates.IDLE,
			() -> Constants.controller.getPOV() == 180
		);
	}

	@Override
	public void periodic() {
		super.periodic();

		intakeSubsystem.setState(getState().getIntakeState());
		ampBarSubsystem.setState(getState().getAmpBarState());
		shooterSubsystem.setState(getState().getShooterState());
		climberSubsystem.setState(getState().getClimberState());
		shooterSubsystem.setManagerState(getState());

		intakeSubsystem.periodic();
		ampBarSubsystem.periodic();
		shooterSubsystem.periodic();
		driveSubsystem.periodic();

		if (useAutoAlign.getSelected() == null ? true : useAutoAlign.getSelected()) {
			autoAlignSubsystem.periodic();
		}

		// Cancel all actions regardless of whats happening
		if (Constants.operatorController.getXButtonPressed()) {
			setState(ManagerStates.IDLE);
		}
	}

	public void stop() {
		intakeSubsystem.stop();
		ampBarSubsystem.stop();
		shooterSubsystem.stop();
		climberSubsystem.stop();
		driveSubsystem.stop();
	}

	@Override
	protected void runState() {
		NoteSimulator.update();
		NoteSimulator.logNoteInfo();
	}
}
