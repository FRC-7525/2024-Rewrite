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
    public double shooterSpeed = 0.0;
    public double shooterAppliedVolts = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setSpeed(double rps) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void configureFF(double kS, double kV, double kA) {}

  public default boolean nearSpeedPoint() {
    return false;
  }
}
