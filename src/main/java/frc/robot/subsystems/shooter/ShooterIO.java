package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {

    public double rightShooterSpeed = 0.0;
    public double leftShooterSpeed = 0.0;
    public double leftShooterAppliedVolts = 0.0;
    public double rightShooterAppliedVolts = 0.0;
    public double shooterSpeedPoint = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setSpeed(double rps) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default boolean nearSpeedPoint() {
    return false;
  }

  public default double getAverageSpeed() {
    return 0.0;
  }
}
