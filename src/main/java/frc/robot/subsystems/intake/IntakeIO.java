package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Pose3d position;

    public double wheelSpeed;
    public double wheelAppliedVoltage;
    public double wheelSpeedpoint;

    public double pivotPosition;
    public double pivotAppliedVoltage;
    public double pivotSetpoint;
    public double pivotSetpointError;

    public boolean usingInPID;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setSetpoints(
      double pivotMotorSetpoint, double intakeMotorSetpoint, boolean useIn) {}

  public default void stop() {}

  public default double getPosition() {
    return 0.0;
  }

  public default void configurePID(PIDConstants outPIDConst, PIDConstants inPIPidConst) {}
}
