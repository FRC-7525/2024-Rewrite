package frc.robot.subsystems.ampBar;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface AmpBarIO {
	@AutoLog
	public static class AmpBarIOInputs {

		public String ampBarState;

		// Pivot
		public double pivotPosition;
		public double pivotSetpoint;
		
		// Spinners
		public double spinnerSpeed;
		public double spinnerSetpoint;
	}

	public static class AmpBarIOOutputs {
		@AutoLogOutput
		public double spinnerAppliedVoltage = 0.0;
		@AutoLogOutput
		public double pivotAppliedVoltage = 0.0;
	}

	public default void updateInput(AmpBarIOInputs inputs) {}

	public default void updateOutputs(AmpBarIOOutputs outputs) {}

	public default void setPivotPosition(double position) {}

	public default void setSpinnerSpeedpoint(double speed) {}

	public default void stop() {}

	public default double getPivotPosition() {
		return 0.0;
	}

	public default double getSpinnerSpeed() {
		return 0.0;
	}

	public default void configurePID(double kP, double kI, double kD) {}

	public default boolean atSetPoint() {
		return false;
	}
}
