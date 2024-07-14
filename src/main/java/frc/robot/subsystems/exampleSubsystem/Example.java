package frc.robot.subsystems.exampleSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import org.littletonrobotics.junction.Logger;

public class Example extends Subsystem<ExampleStates> {

  ExampleIO io;
  ExampleIOInputsAutoLogged inputs;

  public Example(ExampleIO io) {
    super("Example", ExampleStates.OFF);
    this.io = io;
    inputs = new ExampleIOInputsAutoLogged();

    // Have Actual PID values
    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(0.0, 0.0, 0.0);
      case REPLAY:
        io.configurePID(0.0, 0.0, 0.0);
        break;
      case SIM:
        io.configurePID(0.0, 0.0, 0.0);
        break;
      default:
        break;
    }
  }

  protected void runState() {
    io.setSpeed(getState().getSpeedPoint());
  }

  @Override
  public void periodic() {
    super.periodic();

    /* Again, you only need to log 3d poses for pivoting subsystems
    so amp bar and intake. Both of those mechanisms rotate around the
    pitch axis. */
    Logger.recordOutput(
        "Example Pose",
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, io.getPosition(), 0)));

    Logger.processInputs("Example", inputs);
    io.updateInputs(inputs);
  }
}
