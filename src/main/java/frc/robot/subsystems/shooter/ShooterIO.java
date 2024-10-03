package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface ShooterIO {
	@AutoLog
	public static class ShooterIOInputs {

		public double rightShooterSpeed = 0.0;
		public double leftShooterSpeed = 0.0;
		public double shooterSpeedPoint = 0.0;
		public double rightShooterAmp = 0.0;
		public double leftShooterAmp = 0.0;
	}

	public static class ShooterIOOutputs {

		@AutoLogOutput
		public double leftShooterAppliedVolts = 0.0;

		@AutoLogOutput
		public double rightShooterAppliedVolts = 0.0;
	}

	public default void updateInputs(ShooterIOInputs inputs) {}

	public default void updateOutputs(ShooterIOOutputs outputs) {}

	public default void setSpeed(double rps) {}

	public default void stop() {}

	public default boolean nearSpeedPoint() {
		return false;
	}

	public default double getAverageSpeed() {
		return 0.0;
	}
}
