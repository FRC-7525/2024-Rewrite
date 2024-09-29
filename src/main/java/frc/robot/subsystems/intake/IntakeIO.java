package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose3d;
import java.security.PublicKey;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface IntakeIO {
	@AutoLog
	public static class IntakeIOInputs {

		public Pose3d position;

		public double wheelSpeed;
		public double wheelSpeedpoint;

		public double pivotPosition;
		public double pivotSetpoint;
		public double pivotSetpointError;

		public boolean usingInPID;
	}

	public static class IntakeIOOutputs {

		@AutoLogOutput
		public double pivotAppliedVoltage;

		@AutoLogOutput
		public double wheelAppliedVoltage;
	}

	public default void updateInputs(IntakeIOInputs inputs) {}

	public default void updateOutputs(IntakeIOOutputs outputs) {}

	public default void setSetpoints(
		double pivotMotorSetpoint,
		double intakeMotorSetpoint,
		boolean useIn
	) {}

	public default void stop() {}

	public default double getPosition() {
		return 0.0;
	}

	public default boolean nearSetPoint() {
		return false;
	}

	public default boolean nearSpeedPoint() {
		return false;
	}

	public default void configurePID(PIDConstants outPIDConst, PIDConstants inPIDConst) {}

	public default boolean noteDetected() {
		return false;
	}
}
