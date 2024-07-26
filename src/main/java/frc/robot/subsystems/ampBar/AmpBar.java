package frc.robot.subsystems.ampBar;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import org.littletonrobotics.junction.Logger;

public class AmpBar extends Subsystem<AmpBarStates> {

	AmpBarIO io;
	AmpBarIOInputsAutoLogged inputs;

	public AmpBar(AmpBarIO io) {
		super("Amp Bar", AmpBarStates.OFF);
		this.io = io;
		inputs = new AmpBarIOInputsAutoLogged();

		switch (Constants.currentMode) {
			case REAL:
				io.configurePID(1, 0, 0);
				break;
			case SIM:
				io.configurePID(1, 0, 0);
				break;
			case REPLAY:
				io.configurePID(1, 0, 0);
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
			new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, io.getPivotPosition(), 0))
		);
		Logger.processInputs("Amp Bar", inputs);
		io.updateInput(inputs);
	}
}
