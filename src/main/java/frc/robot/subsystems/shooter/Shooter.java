package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends Subsystem<ShooterStates> {

  private ShooterIO io;
  ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io) {
    super("Shooter", ShooterStates.OFF);
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();

    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(
            Constants.Shooter.REAL_PID.kP,
            Constants.Shooter.REAL_PID.kI,
            Constants.Shooter.REAL_PID.kD);
        break;
      case REPLAY:
        io.configurePID(0.0, 0.0, 0.0);
        break;
      case SIM:
        io.configurePID(
            Constants.Shooter.SIM_PID.kP,
            Constants.Shooter.SIM_PID.kI,
            Constants.Shooter.SIM_PID.kD);
        break;
      default:
        break;
    }
  }

  @Override
  protected void runState() {
    io.setSpeed(getState().getMotorSpeedpoint());
  }

  public boolean nearSpeedPoint() {
    return io.nearSpeedPoint();
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
