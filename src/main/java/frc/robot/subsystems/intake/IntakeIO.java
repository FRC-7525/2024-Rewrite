package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public Pose3d position;
        public String stateString;

        public double wheelSpeed;
        public double wheelAppliedVoltage;
        public double wheelSpeedpoint;

        public double pivotPosition; //Not sure what exact values I need to store here (Pose? Translation?)
        public double pivotAppliedVoltage;
        public double pivotSetpoint; //Also not sure what value the setpoint is supposed to be
        public double pivotSetpointError; //Same problems
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setSetpoints(double pivotMotorSetpoint, double intakeMotorSetpoint, boolean useIn) {}
    
    public default void stop() {}

    public default double getPosition() {
        return 0.0;
    }

    public default void configurePID(double kP, double kI, double kD) {}
}
