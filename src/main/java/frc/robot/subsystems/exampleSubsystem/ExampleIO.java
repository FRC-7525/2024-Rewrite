package frc.robot.subsystems.exampleSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ExampleIO {

  @AutoLog
  public static class ExampleIOInputs {
    public double wheelSpeed = 0.0;
    public double wheelSpeedPoint = 0.0;
    public double wheelAppliedVolts = 0.0;
  }

  public default void updateInputs(ExampleIOInputs input) {}

  public default void setSpeed(double speed) {}

  public default double getPosition() {
    return 0.0;
  }

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
