package frc.robot.subsystems.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import org.littletonrobotics.junction.Logger;

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

    addTrigger(
        AutoAlignStates.OFF,
        AutoAlignStates.SOURCE_SPEAKER,
        () -> Constants.operatorController.getLeftBumper());
    addTrigger(
        AutoAlignStates.OFF,
        AutoAlignStates.AMP_SPEAKER,
        () -> Constants.operatorController.getRightBumper());
    addTrigger(
        AutoAlignStates.OFF, AutoAlignStates.AMP, () -> Constants.operatorController.getAButton());
  }

  @Override
  protected void runState() {
    if (!(getState() == AutoAlignStates.OFF)) {
      io.lockDrive();
      io.setTargetPose(
          (DriverStation.getAlliance().get() == Alliance.Red)
              ? getState().getTargetPose2dRed()
              : getState().getTargetPose2dBlue());
      io.driveToTargetPose();

      if (io.nearTargetPoint()) {
        io.returnDriveToNormal();
        setState(AutoAlignStates.OFF);
      }
    }

    Logger.recordOutput("testing", getState());

    if (Constants.operatorController.getXButtonPressed()) {
      setState(AutoAlignStates.OFF);
    }
  }

  public void setAutoAlignTarget(Pose2d targetPose2d) {
    io.setTargetPose(targetPose2d);
  }
}
