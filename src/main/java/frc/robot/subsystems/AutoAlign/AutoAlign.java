package frc.robot.subsystems.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import org.littletonrobotics.junction.Logger;

public class AutoAlign extends Subsystem<AutoAlignStates> {

	AutoAlignIO io;

	public AutoAlign(AutoAlignIO io) {
		super("AutoAlign", AutoAlignStates.OFF);
		this.io = io;

		/* Sets PID values for each mode */
		switch (Constants.currentMode) {
			case REAL:
				io.configureTranslationalPID(0, 0, 0);
				io.configurerotationalPID(0, 0, 0);
				break;
			case SIM:
				io.configureTranslationalPID(
					Constants.AutoAlign.TRANSLATIONAL_PID.kP,
					Constants.AutoAlign.TRANSLATIONAL_PID.kI,
					Constants.AutoAlign.TRANSLATIONAL_PID.kD
				);
				io.configurerotationalPID(
					Constants.AutoAlign.ROTATIONAL_PID.kP,
					Constants.AutoAlign.ROTATIONAL_PID.kI,
					Constants.AutoAlign.ROTATIONAL_PID.kD
				);
				break;
			case REPLAY:
				io.configureTranslationalPID(0, 0, 0);
				io.configurerotationalPID(0, 0, 0);
				break;
		}

		/* Create triggers for reading controller inputs */
		addTrigger(AutoAlignStates.OFF, AutoAlignStates.SOURCE_SPEAKER, () ->
			Constants.operatorController.getLeftBumper()
		);
		addTrigger(AutoAlignStates.OFF, AutoAlignStates.AMP_SPEAKER, () ->
			Constants.operatorController.getRightBumper()
		);
		addTrigger(AutoAlignStates.OFF, AutoAlignStates.AMP, () ->
			Constants.operatorController.getAButton()
		);
	}

	@Override
	protected void runState() {
		/* sets drive to auto align and drives to target pose*/
		if (!(getState() == AutoAlignStates.OFF)) {
			io.lockDrive();
			io.setTargetPose(
				(DriverStation.getAlliance().get() == Alliance.Red)
					? getState().getTargetPose2dRed()
					: getState().getTargetPose2dBlue()
			);
			io.driveToTargetPose();

			/*returns controls to driver once at target  */
			if (io.nearTargetPoint()) {
				io.setTargetPose(new Pose2d());
				io.returnDriveToNormal();
				setState(AutoAlignStates.OFF);
			}
		}
		/*X button to abort*/
		if (Constants.operatorController.getXButtonPressed()) {
			setState(AutoAlignStates.OFF);
		}

		Logger.recordOutput("AutoAlign/Target Pose 2d", io.getTargetPose2d());
		Logger.recordOutput("AutoAlign/Applied X", io.getAppliedX());
		Logger.recordOutput("AutoAlign/Applied Y", io.getAppliedY());
		Logger.recordOutput("AutoAlign/Applied Rotational", io.getAppliedRotational());
	}
}