package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.manager.Manager;
import frc.robot.subsystems.manager.ManagerStates;

public class AutoAlign {
  private static PIDController xPIDController =
      new PIDController(
          Constants.AutoAlign.TRANSLATIONAL_PID.kP,
          Constants.AutoAlign.TRANSLATIONAL_PID.kI,
          Constants.AutoAlign.TRANSLATIONAL_PID.kD);
  private static PIDController yPIDController =
      new PIDController(
          Constants.AutoAlign.TRANSLATIONAL_PID.kP,
          Constants.AutoAlign.TRANSLATIONAL_PID.kI,
          Constants.AutoAlign.TRANSLATIONAL_PID.kD);
  private static final PIDController rotationalPIDController =
      new PIDController(
          Constants.AutoAlign.ROTATIONAL_PID.kP,
          Constants.AutoAlign.ROTATIONAL_PID.kI,
          Constants.AutoAlign.ROTATIONAL_PID.kD);
  private static Pose2d targetPose2d;
  private static AutoAlignInstruction autoAlignInstruction;
  private static Timer timer = new Timer();
  private static boolean setStateAlready;

  private static Drive drive;
  private static Manager manager;

  public static void setTargetPose(Pose2d target) {
    targetPose2d = target;
  }

  public static void setDrive(Drive inputDrive) {
    drive = inputDrive;
  }

  public static void setManager(Manager inputManager) {
    manager = inputManager;
  }

  // calculate chassissSpeed obj
  public static void calculateChassisSpeed() {
    Pose2d currentPose2d = drive.getPose();

    if (!nearSetPoint()) {
      manager.setState(autoAlignInstruction.duringDrive);

      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xPIDController.calculate(currentPose2d.getX(), targetPose2d.getX()),
              yPIDController.calculate(currentPose2d.getY(), targetPose2d.getY()),
              rotationalPIDController.calculate(
                  currentPose2d.getRotation().getRadians(),
                  targetPose2d.getRotation().getRadians()),
              drive.getRotation()));
    } else {
      // this is the only way i could think of to make the code pretty. please don't murder me
      // Amp bar needs time to be able to be fed
      if (timer.hasElapsed(Constants.AutoAlign.TIME_FOR_SCORING) && setStateAlready) {
        manager.setState(ManagerStates.IDLE);
        drive.setState(DriveStates.REGULAR_DRIVE);

      } else if (autoAlignInstruction == AutoAlignInstruction.SCORE_AMP_BAR
          && timer.hasElapsed(Constants.AutoAlign.TIME_TO_FEED)) {
        manager.setState(autoAlignInstruction.atSetPoint);

      } else if (autoAlignInstruction == AutoAlignInstruction.SHOOT
          && timer.hasElapsed(Constants.AutoAlign.TIME_TO_FEED)) {
        manager.setState(autoAlignInstruction.atSetPoint);
      }

      if (!setStateAlready) {
        setStateAlready = true;
        timer.reset();
        timer.start();
      }
    }
  }

  public static boolean nearSetPoint() {
    Pose2d currentPose2d = drive.getPose();

    if (Math.abs(currentPose2d.getX() - targetPose2d.getX())
            < Constants.AutoAlign.TRANSLATION_ERROR_MARGIN
        && Math.abs(currentPose2d.getY() - targetPose2d.getY())
            < Constants.AutoAlign.TRANSLATION_ERROR_MARGIN
        && Math.abs(
                currentPose2d.getRotation().getDegrees() - targetPose2d.getRotation().getDegrees())
            < Constants.AutoAlign.ROTATION_ERROR_MARGIN) {
      return true;
    }
    return false;
  }

  public static void periodic() {
    if (Constants.operatorController.getLeftBumper()) {
      setTargetPose(
          (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
              ? Constants.AutoAlign.redSourceSpeakerPose
              : Constants.AutoAlign.blueAmpSpeakerPose);
      autoAlignInstruction = AutoAlignInstruction.SHOOT;
      setStateAlready = false;
      drive.setState(DriveStates.AUTO_ALIGN);
    } else if (Constants.operatorController.getRightBumper()) {
      setTargetPose(
          (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
              ? Constants.AutoAlign.redAmpSpeakerPose
              : Constants.AutoAlign.blueSourceSpeakerPose);
      autoAlignInstruction = AutoAlignInstruction.SHOOT;
      setStateAlready = false;
      drive.setState(DriveStates.AUTO_ALIGN);
    } else if (Constants.operatorController.getAButton()) {
      setTargetPose(
          (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
              ? Constants.AutoAlign.redAmpPose
              : Constants.AutoAlign.blueAmpPose);
      autoAlignInstruction = AutoAlignInstruction.SCORE_AMP_BAR;
      timer.restart();
      timer.start();
      setStateAlready = false;
      drive.setState(DriveStates.AUTO_ALIGN);
    } else if (Constants.operatorController.getXButton()) {
      drive.setState(DriveStates.REGULAR_DRIVE);
    }
  }
  // #TODO add drive to amp bar and auto shoot.
  // create timer for feeding and holding
}