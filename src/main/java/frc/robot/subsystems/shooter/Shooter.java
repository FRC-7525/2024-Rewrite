package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.manager.ManagerStates;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs;
import frc.robot.util.NoteSimulator;
import org.littletonrobotics.junction.Logger;

public class Shooter extends Subsystem<ShooterStates> {

	private ShooterIO io;
	private ManagerStates managerState;
	ShooterIOInputsAutoLogged inputs;
	private ShooterIOOutputs outputs;

	public Shooter(ShooterIO io) {
		super("Shooter", ShooterStates.OFF);
		this.io = io;
		inputs = new ShooterIOInputsAutoLogged();
		outputs = new ShooterIOOutputs();

		switch (Constants.currentMode) {
			case REAL:
				break;
			case REPLAY:
				break;
			case SIM:
				break;
			default:
				break;
		}
	}

	@Override
	protected void runState() {
		io.setSpeed(getState().getMotorSpeedpoint());

		if (managerState == ManagerStates.SHOOTING) {
			// NoteSimulator.launch(
			// 	io.getAverageSpeed() * Constants.Shooter.CIRCUMFRENCE_OF_SHOOTER_SPINNER
			// );
		}
	}

	public boolean nearSpeedPoint() {
		return io.nearSpeedPoint();
	}

	public void stop() {
		io.stop();
	}

	@Override
	public void periodic() {
		super.periodic();
		io.updateInputs(inputs);
		io.updateOutputs(outputs);
		Logger.processInputs("Shooter", inputs);
	}

	public void setManagerState(ManagerStates state) {
		managerState = state;
	}
}
