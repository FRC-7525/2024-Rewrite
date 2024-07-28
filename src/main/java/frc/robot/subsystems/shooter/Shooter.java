package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.manager.ManagerStates;
import frc.robot.util.NoteSimulator;
import org.littletonrobotics.junction.Logger;

public class Shooter extends Subsystem<ShooterStates> {
  private ShooterIO io;
  private ManagerStates managerState;
  ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io) {
    super("Shooter", ShooterStates.OFF);
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();

    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(1.0, 0, 0);
        break;
      case REPLAY:
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        io.configurePID(.35, 0.0, 0.0);
        break;
      default:
        break;
    }
  }

  @Override
  protected void runState() {
    io.setSpeed(getState().getMotorSpeedpoint());

    if (managerState == ManagerStates.SHOOTING) {
      NoteSimulator.launch(
          io.getAverageSpeed() * Constants.Shooter.CIRCUMFRENCE_OF_SHOOTER_SPINNER);
    }
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

  public void setManagerState(ManagerStates state) {
    managerState = state;
  }
}
