package frc.robot.subsystems.AutoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveStates;

public class AutoAlignIOSim implements AutoAlignIO {

  Drive driveSubsystem;
  PIDController xPIDController;
  PIDController yPIDController;
  PIDController rotationalPIDController;

  Pose2d targetPose2d;

  public AutoAlignIOSim(Drive driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    xPIDController = new PIDController(0, 0, 0);
    yPIDController = new PIDController(0, 0, 0);
    rotationalPIDController = new PIDController(0, 0, 0);

    targetPose2d = new Pose2d();
  }

  @Override
  public void setTargetPose(Pose2d target) {
    targetPose2d = target;
  }

  @Override
  public void configurexPID(double kP, double kI, double kD) {
    xPIDController.setPID(kP, kI, kD);
  }

  @Override
  public void configureyPID(double kP, double kI, double kD) {
    yPIDController.setPID(kP, kI, kD);
  }

  @Override
  public void configurerotationalPID(double kP, double kI, double kD) {
    rotationalPIDController.setPID(kP, kI, kD);
  }

  public boolean nearTargetPoint() {
    Pose2d currentPose2d = driveSubsystem.getPose();

    return (Math.abs(currentPose2d.getX() - targetPose2d.getX())
            < Constants.AutoAlign.TRANSLATION_ERROR_MARGIN
        && Math.abs(currentPose2d.getY() - targetPose2d.getY())
            < Constants.AutoAlign.TRANSLATION_ERROR_MARGIN
        && Math.abs(
                currentPose2d.getRotation().getDegrees() - targetPose2d.getRotation().getDegrees())
            < Constants.AutoAlign.ROTATION_ERROR_MARGIN);
  }

  @Override
  public void driveToTargetPose() {
    Pose2d currentPose2d = driveSubsystem.getPose();

    driveSubsystem.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xPIDController.calculate(currentPose2d.getX(), targetPose2d.getX()),
            yPIDController.calculate(currentPose2d.getY(), targetPose2d.getY()),
            rotationalPIDController.calculate(
                currentPose2d.getRotation().getRadians(), targetPose2d.getRotation().getRadians()),
            driveSubsystem.getRotation()));
  }

  @Override
  public void returnDriveToNormal() {
    driveSubsystem.setState(DriveStates.REGULAR_DRIVE);
  }
}
