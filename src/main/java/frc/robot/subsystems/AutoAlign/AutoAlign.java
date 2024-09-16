package frc.robot.subsystems.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class AutoAlign extends Subsystem<AutoAlignStates> {

  AutoAlignIO io;

  public AutoAlign(AutoAlignIO io) {
    super("AutoAlign", AutoAlignStates.OFF);

    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        io.configurexPID(0, 0, 0);
        io.configureyPID(0, 0, 0);
        io.configurerotationalPID(0, 0, 0);
        break;
      case SIM:
        io.configurexPID(
            Constants.AutoAlign.TRANSLATIONAL_PID.kP,
            Constants.AutoAlign.TRANSLATIONAL_PID.kI,
            Constants.AutoAlign.TRANSLATIONAL_PID.kD);
        io.configureyPID(
            Constants.AutoAlign.TRANSLATIONAL_PID.kP,
            Constants.AutoAlign.TRANSLATIONAL_PID.kI,
            Constants.AutoAlign.TRANSLATIONAL_PID.kD);
        io.configurerotationalPID(
            Constants.AutoAlign.ROTATIONAL_PID.kP,
            Constants.AutoAlign.ROTATIONAL_PID.kI,
            Constants.AutoAlign.ROTATIONAL_PID.kD);
        break;
      case REPLAY:
        io.configurexPID(0, 0, 0);
        io.configureyPID(0, 0, 0);
        io.configurerotationalPID(0, 0, 0);
        break;
    }
  }

  @Override
  protected void runState() {
    if (getState() == AutoAlignStates.ON) {
      io.driveToTargetPose();

      if (io.nearTargetPoint()) {
        io.returnDriveToNormal();
      }
    }
  }

  public void setAutoAlignTarget(Pose2d targetPose2d) {
    io.setTargetPose(targetPose2d);
  }
}
