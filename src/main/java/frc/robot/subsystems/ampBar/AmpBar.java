package frc.robot.subsystems.ampBar;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.ampBar.AmpBarIO.AmpBarIOOutputs;
import org.littletonrobotics.junction.Logger;

public class AmpBar extends Subsystem<AmpBarStates> {

	AmpBarIO io;
	AmpBarIOInputsAutoLogged inputs;
	AmpBarIOOutputs outputs;

	public AmpBar(AmpBarIO io) {
		super("Amp Bar", AmpBarStates.OFF);
		this.io = io;
		inputs = new AmpBarIOInputsAutoLogged();
		outputs = new AmpBarIOOutputs();

		switch (Constants.currentMode) {
			case REAL:
				io.configurePID(
					Constants.AmpBar.REAL_PID.kP,
					Constants.AmpBar.REAL_PID.kI,
					Constants.AmpBar.REAL_PID.kD
				);
				break;
			case SIM:
				io.configurePID(
					Constants.AmpBar.SIM_PID.kP,
					Constants.AmpBar.SIM_PID.kP,
					Constants.AmpBar.SIM_PID.kP
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
	protected void runState() {
		io.setSpinnerSpeedpoint(getState().getMotorSpeedpoint());
		io.setPivotPosition(getState().getPivotPositionSetpoint());
	}

	public void stop() {
		io.stop();
	}

	@Override
	public void periodic() {
		super.periodic();

		Logger.recordOutput(
			"Amp Bar/Amp Bar Pose3d",
			new Pose3d(
				Constants.AmpBar.ZEROED_PIVOT_TRANSLATION,
				new Rotation3d(0, io.getPivotPosition(), 0)
			)
		);
		Logger.processInputs("Amp Bar", inputs);
		io.updateInput(inputs);
		io.updateOutputs(outputs);
	}
}
