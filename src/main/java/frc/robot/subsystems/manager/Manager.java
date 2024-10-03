package frc.robot.subsystems.manager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.AprilTagVision.Vision;
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
	private Vision vision;

	private SendableChooser<Boolean> useBeamBreaks;
	private SendableChooser<Boolean> useAutoAlign;
	private SendableChooser<Boolean> driverShooterAfterSpinning;
	private SendableChooser<Boolean> useClimbers;
	private SendableChooser<Boolean> useVision;
	private SendableChooser<Boolean> useHeadingCorrection;

	private Boolean useBeamBreaksVal;
	private Boolean useAutoAlignVal;
	private Boolean driverShooterAfterSpinningVal;
	private Boolean useClimbersVal;
	private Boolean useVisionVal;
	private Boolean useHeadingCorrectionVal;

	private final int SENDABLE_CHECK_INTERVAL = 50;
	private int loopCounter;

	public Manager() {
		super("Manager", ManagerStates.IDLE);
		// NoteSimulator.setDrive(driveSubsystem);

		// Setup toggles for dangerous stuff
		useBeamBreaks = new SendableChooser<>();
		useAutoAlign = new SendableChooser<>();
		useClimbers = new SendableChooser<>();
		useVision = new SendableChooser<>();
		useHeadingCorrection = new SendableChooser<>();
		driverShooterAfterSpinning = new SendableChooser<>();

		loopCounter = 50;

		useBeamBreaksVal = false;
		useAutoAlignVal = false;
		driverShooterAfterSpinningVal = false;
		useClimbersVal = false;
		useVisionVal = false;
		useHeadingCorrectionVal = false;

		useBeamBreaks.setDefaultOption("On", true);
		useBeamBreaks.addOption("Off", false);

		useAutoAlign.setDefaultOption("On", true);
		useAutoAlign.addOption("Off", false);

		driverShooterAfterSpinning.setDefaultOption("On", true);
		driverShooterAfterSpinning.addOption("Off", false);

		useClimbers.setDefaultOption("On", true);
		useClimbers.addOption("Off", false);

		useVision.addOption("Off", false);
		useVision.addOption("On", true);

		useHeadingCorrection.setDefaultOption("On", true);
		useHeadingCorrection.addOption("Off", false);

		SmartDashboard.putData("Beam Breaks Toggle", useBeamBreaks);
		SmartDashboard.putData("Auto Align Toggle", useAutoAlign);
		SmartDashboard.putData("Climbers Toggle", useClimbers);
		SmartDashboard.putData("AT Vision Toggle", useVision);
		SmartDashboard.putData("Heading Correction", useHeadingCorrection);
		SmartDashboard.putData("Drive Shoot After Spinning Toggle", driverShooterAfterSpinning);

		// Subsystem configs
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
				useVision.setDefaultOption("On", true);
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
				useVision.setDefaultOption("Off", false);
				break;
			default:
				break;
		}
		autoAlignSubsystem = new AutoAlign(new AutoAlignIO(driveSubsystem));
		vision = new Vision(driveSubsystem);

		// NoteSimulator.setDrive(driveSubsystem);

		/*
		 * Generally each action has a specific button (Intaking, Shooting, etc.)
		 * In intermediary states X will return to idle
		 * Specific button will transition through the states
		 * Some transitions are automatic with timers or sensors
		 */

		// Intaking (B)
		addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING, () ->
			Constants.controller.getBButtonPressed()
		);
		addTrigger(
			ManagerStates.INTAKING,
			ManagerStates.IDLE,
			() ->
				(intakeSubsystem.noteDetected() &&
					intakeSubsystem.nearSetPoint() &&
					useBeamBreaksVal) ||
				Constants.controller.getBButtonPressed()
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
				(ampBarSubsystem.noteDetected() && useBeamBreaksVal) ||
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
			() -> driverShooterAfterSpinningVal || Constants.controller.getAButtonPressed()
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

		// Climbing (POV)
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.STAGING_FOR_CLIMB,
			() -> Constants.controller.getPOV() == 0 && useClimbersVal
		);
		addTrigger(ManagerStates.STAGING_FOR_CLIMB, ManagerStates.CLIMBING, () ->
			ampBarSubsystem.nearSetPoints()
		);
		addTrigger(
			ManagerStates.CLIMBING,
			ManagerStates.CLIMBED,
			() -> Constants.controller.getPOV() == 180
		);
		addTrigger(ManagerStates.CLIMBED, ManagerStates.IDLE, () ->
			Constants.controller.getXButtonPressed()
		);
	}

	@Override
	public void periodic() {
		super.periodic();

		loopCounter += 1;

		intakeSubsystem.setState(getState().getIntakeState());
		ampBarSubsystem.setState(getState().getAmpBarState());
		shooterSubsystem.setState(getState().getShooterState());
		shooterSubsystem.setManagerState(getState());

		intakeSubsystem.periodic();
		ampBarSubsystem.periodic();
		shooterSubsystem.periodic();
		driveSubsystem.periodic();
		climberSubsystem.periodic();

		// Set Sendable Stuff:
		if (loopCounter % SENDABLE_CHECK_INTERVAL == 0) {
			useBeamBreaksVal = useBeamBreaks.getSelected() == null
				? true
				: useBeamBreaks.getSelected();
			useAutoAlignVal = useAutoAlign.getSelected() == null
				? true
				: useAutoAlign.getSelected();
			driverShooterAfterSpinningVal = driverShooterAfterSpinning.getSelected() == null
				? true
				: driverShooterAfterSpinning.getSelected();
			useClimbersVal = useClimbers.getSelected() == null ? true : useClimbers.getSelected();
			useVisionVal = useVision.getSelected() == null ? true : useVision.getSelected();
			useHeadingCorrectionVal = useHeadingCorrection.getSelected() == null
				? true
				: useHeadingCorrection.getSelected();
		}

		// AA
		if (useAutoAlignVal) {
			autoAlignSubsystem.periodic();
			if (autoAlignSubsystem.nearTargetPoint()) {
				switch (autoAlignSubsystem.getCachedState()) {
					case AMP:
						setState(ManagerStates.STAGING_AMP);
						break;
					case AMP_SPEAKER:
						setState(ManagerStates.SPINNING_UP);
						break;
					case SOURCE_SPEAKER:
						setState(ManagerStates.SPINNING_UP);
						break;
					case OFF:
						break;
					default:
						break;
				}
			}
		}

		if (useClimbersVal) {
			climberSubsystem.periodic();
			climberSubsystem.setState(getState().getClimberState());
		}

		if (useVisionVal) vision.periodic();

		driveSubsystem.toggleHeadingCorrection(useHeadingCorrectionVal);

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
		// NoteSimulator.update();
		// NoteSimulator.logNoteInfo();
	}

	// Auto Stuff
	public void setState(ManagerStates state) {
		setState(state);
	}

	// Near Setpoint functions for stuff
	public boolean intakeNearSetpoints() {
		return intakeSubsystem.nearSetpoints();
	}

	public boolean ampBarNearSetpoints() {
		return ampBarSubsystem.nearSetPoints();
	}

	public boolean shooterNearSpeedPoint() {
		return shooterSubsystem.nearSpeedPoint();
	}

	public boolean driveNearSetPose(Pose2d targetPose) {
		return driveSubsystem.nearSetPose(targetPose);
	}
}
