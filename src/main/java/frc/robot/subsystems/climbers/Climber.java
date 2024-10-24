package frc.robot.subsystems.climbers;

import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.climbers.ClimberIO.ClimberIOOutputs;
import org.littletonrobotics.junction.Logger;

public class Climber extends Subsystem<ClimberStates> {

	private ClimberIO io;
	private ClimberIOInputsAutoLogged inputs;
	private ClimberIOOutputs outputs;

	private boolean zeroed;

	public Climber(ClimberIO io) {
		super("Climber", ClimberStates.OFF);
		this.io = io;
		inputs = new ClimberIOInputsAutoLogged();
		outputs = new ClimberIOOutputs();
		zeroed = false;

		switch (Constants.currentMode) {
			case REAL:
				io.configurePID(
					Constants.Climber.REAL_PID.kP,
					Constants.Climber.REAL_PID.kI,
					Constants.Climber.REAL_PID.kD
				);
				break;
			case SIM:
				io.configurePID(
					Constants.Climber.SIM_PID.kP,
					Constants.Climber.SIM_PID.kI,
					Constants.Climber.SIM_PID.kD
				);
				break;
			case REPLAY:
				io.configurePID(0, 0, 0);
				break;
			default:
				break;
		}
	}

	@Override
	public void runState() {
		Logger.processInputs("Climber", inputs);
		Logger.recordOutput("Climber/Zeroed", zeroed);

		io.updateOutputs(outputs);
		io.updateInputs(inputs);

		if (zeroed) {
			io.setSetpoints(getState().getLeftSetpoint(), getState().getRightSetpoint());
		} else {
			io.zeroClimbers();
			zeroed = io.climbersZeroed();
		}
	}

	public void stop() {
		io.stop();
	}

	public boolean nearSetpoints() {
		return io.nearSetpoints();
	}
}
