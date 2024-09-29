package frc.robot.subsystems.climbers;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {

		public double leftClimberSpeed = 0.0;
		public double rightClimberSpeed = 0.0;

		public double leftClimberPosition = 0.0;
		public double rightClimberPosition = 0.0;

		public double leftClimberSetpoint = 0.0;
		public double rightClimberSetpoint = 0.0;
	}

	public static class ClimberIOOutputs {

		@AutoLogOutput
		public double leftClimberAppliedVoltage = 0.0;

		@AutoLogOutput
		public double rightClimberAppliedVoltage = 0.0;
	}

	public default void setSetpoints(double leftSetpoint, double rightSetpoint) {}

	public default void configurePID(double kP, double kI, double kD) {}

	public default void stop() {}

	public default void zeroClimbers() {}

	public default void updateInputs(ClimberIOInputs inputs) {}

	public default void updateOutputs(ClimberIOOutputs outputs) {}

	public default boolean nearSetpoints() {
		return false;
	}

	public default boolean climbersZeroed() {
		return false;
	}
}
