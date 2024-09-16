package frc.robot.subsystems.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;

public interface AutoAlignIO {

  public static class AutoAlignIOInputs {

    Pose2d targetPose2d;
  }

  public void driveToTargetPose();

  public boolean nearTargetPoint();

  public void setTargetPose(Pose2d target);

  public void returnDriveToNormal();

  public void configurexPID(double kP, double kI, double kD);

  public void configureyPID(double kP, double kI, double kD);

  public void configurerotationalPID(double kP, double kI, double kD);
}
