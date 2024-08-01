package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class AutoAlign {
  private static PIDController xPIDController = Constants.AutoAlign.X_VELOCITY_CONTROLLER;
  private static PIDController yPIDController = Constants.AutoAlign.Y_VELOCITY_CONTROLLER;
  private static final PIDController rotationalPIDController =
      Constants.AutoAlign.ROTATIONAL_VELOCITY_CONTROLLER;
  private static Pose2d targetPose2d = Constants.AutoAlign.redAmpPose;

  private static Drive drive;

  // set target pose

  public static void setTargetPose(Pose2d target) {
    targetPose2d = target;
  }

  public static void setDrive(Drive inputDrive) {
    drive = inputDrive;
  }

  // calculate chassissSpeed obj
  public static void calculateChassisSpeed() {
    Pose2d currentPose2d = drive.getPose();

    if (!nearSetPoint()) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xPIDController.calculate(currentPose2d.getX(), targetPose2d.getX()),
              yPIDController.calculate(currentPose2d.getY(), targetPose2d.getY()),
              rotationalPIDController.calculate(
                  currentPose2d.getRotation().getRadians(),
                  targetPose2d.getRotation().getRadians()),
              drive.getRotation()));
    } else {
      drive.setState(DriveStates.REGULAR_DRIVE);
    }
  }
  ;

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
      drive.setState(DriveStates.AUTO_ALIGN);
    } else if (Constants.operatorController.getRightBumper()) {
      setTargetPose(
          (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
              ? Constants.AutoAlign.redAmpSpeakerPose
              : Constants.AutoAlign.blueSourceSpeakerPose);
      drive.setState(DriveStates.AUTO_ALIGN);
    }
  }
}
