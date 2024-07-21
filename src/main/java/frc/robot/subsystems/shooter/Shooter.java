package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends Subsystem<ShooterStates> {

  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io) {
    super("Shooter", ShooterStates.OFF);
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(1.0, 0, 0);
        break;
      case REPLAY:
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        break;
    }
  }

  @Override
  protected void runState() {
    super.periodic();
    io.setSpeed(getState().getMotorSpeedpoint());
    Logger.recordOutput(
        "Shooter Pose3d", new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));
    Logger.processInputs("Amp Bar", inputs);
    io.updateInputs(inputs);
  }

  public boolean nearSpeedPoint() {
    return io.nearSpeedPoint();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
