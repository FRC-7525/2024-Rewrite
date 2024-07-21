package frc.robot.subsystems.ampBar;

import org.littletonrobotics.junction.AutoLog;

public interface AmpBarIO {

  @AutoLog
  public static class AmpBarIOInputs {
    public String ampBarState;

    // Pivot
    public double pivotPosition;
    public double pivotSetpoint;
    public double pivotAppliedVoltage;
    // Spinners
    public double spinnerSpeed;
    public double spinnerSetpoint;
    public double spinnerAppliedVoltage;
  }

  public void updateInput(AmpBarIOInputs inputs);

  public void setPivotPosition(double position);

  public void setSpinnerSpeedpoint(double speed);

  public double getPivotPosition();

  public double getSpinnerSpeed();

  public void configurePID(double kP, double kI, double kD);

  public boolean atSetPoint();
}
