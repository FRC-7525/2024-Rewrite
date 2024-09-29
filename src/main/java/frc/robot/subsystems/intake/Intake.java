package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.util.NoteSimulator;
import org.littletonrobotics.junction.Logger;

public class Intake extends Subsystem<IntakeStates> {

	IntakeIO io;
	IntakeIOInputsAutoLogged inputs;
	IntakeIOOutputs outputs;

	public Intake(IntakeIO io) {
		super("Intake", IntakeStates.OFF);
		this.io = io;
		inputs = new IntakeIOInputsAutoLogged();
		outputs = new IntakeIOOutputs();

		// Configure PIDs here
		switch (Constants.currentMode) {
			case REAL:
				io.configurePID(Constants.Intake.REAL_OUT_PID, Constants.Intake.REAL_IN_PID);
				break;
			case REPLAY:
				io.configurePID(new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0));
				break;
			case SIM:
				io.configurePID(Constants.Intake.REAL_OUT_PID, Constants.Intake.REAL_IN_PID);
				break;
			default:
				break;
		}
	}

	protected void runState() {
		io.setSetpoints(
			getState().getPivotSetPoint(),
			getState().getMotorSetPoint(),
			getState().getUsingPID()
		);
		if (getState() == IntakeStates.INTAKING) {
			NoteSimulator.attachToShooter();
		}
	}

	public void stop() {
		io.stop();
	}

	public boolean nearSetpoints() {
		return io.nearSetPoint() && io.nearSpeedPoint();
	}

	@Override
	public void periodic() {
		super.periodic();

		Logger.recordOutput(
			"Intake/Intake Pose",
			new Pose3d(
				Constants.Intake.ZEROED_PIVOT_TRANSLATION,
				new Rotation3d(0, io.getPosition(), 0)
			)
		);

		Logger.processInputs("Intake", inputs);
		io.updateInputs(inputs);
		io.updateOutputs(outputs);
		Logger.recordOutput("Intake BB", io.noteDetected());
	}

	public boolean noteDetected() {
		return io.noteDetected();
	}

	public boolean nearSetPoint() {
		return io.nearSetPoint();
	}

	public boolean nearSpeedPoint() {
		return io.nearSpeedPoint();
	}
}
