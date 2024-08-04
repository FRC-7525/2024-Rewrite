package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.util.NoteSimulator;
import org.littletonrobotics.junction.Logger;

public class Intake extends Subsystem<IntakeStates> {
  IntakeIO io;
  IntakeIOInputsAutoLogged inputs;

  public Intake(IntakeIO io) {
    super("Intake", IntakeStates.OFF);
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();

    // Configure PIDs here
    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(new PIDConstants(1, 0, 0), new PIDConstants(1, 0, 0));
        break;
      case REPLAY:
        io.configurePID(new PIDConstants(1, 0, 0), new PIDConstants(1, 0, 0));
        break;
      case SIM:
        io.configurePID(new PIDConstants(1, 0, 0), new PIDConstants(1, 0, 0));
        break;
      default:
        break;
    }
  }

  protected void runState() {
    io.setSetpoints(
        getState().getPivotSetPoint(), getState().getMotorSetPoint(), getState().getUsingPID());
    if (getState() == IntakeStates.INTAKING) {
      NoteSimulator.attachToShooter();
    }
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        "Intake/Intake Pose",
        new Pose3d(
            Constants.Intake.ZEROED_PIVOT_TRANSLATION, new Rotation3d(0, io.getPosition(), 0)));

    Logger.processInputs("Intake", inputs);
    io.updateInputs(inputs);
  }
}
