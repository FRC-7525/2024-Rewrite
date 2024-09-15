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

		@AutoLogOutput
		public double spinnerAppliedVoltage;
		@AutoLogOutput
		public double pivotAppliedVoltage;
	}

	public default void updateInput(AmpBarIOInputs inputs) {}

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
