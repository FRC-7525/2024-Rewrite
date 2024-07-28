package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class NoteSimulator {
  private static Drive drive;

  private static Pose3d currentFieldPose = new Pose3d();
  private static Translation3d fieldVelocity = new Translation3d();
  private static boolean inShooter = false;
  private static List<Translation3d> noteTrajectory = new ArrayList<>();

  public static void setDrive(Drive drivesystem) {
    drive = drivesystem;
  }

  public static void attachToShooter() {
    inShooter = true;
    noteTrajectory.clear();
  }

  public static boolean isAttached() {
    return inShooter;
  }

  public static List<Translation3d> getNoteTrajectory() {
    return noteTrajectory;
  }

  public static void launch(double velocity) {
    if (!inShooter) {
      return;
    }

    Logger.recordOutput("Launch Velocity", velocity);

    currentFieldPose = getFieldPose(Constants.NoteSim.SHOOTER_POSE3D);
    inShooter = false;

    fieldVelocity = new Translation3d(velocity, currentFieldPose.getRotation());

    ChassisSpeeds robotVel = new ChassisSpeeds(0, 0, 0); // assumes robot is not moving
    ChassisSpeeds fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, drive.getRotation());

    fieldVelocity =
        fieldVelocity.plus(
            new Translation3d(fieldRel.vxMetersPerSecond, fieldRel.vyMetersPerSecond, 0.0));
  }

  public static Pose3d getFieldPose(Pose3d shooterPose) {
    if (inShooter) {
      return new Pose3d(drive.getPose())
          .transformBy(new Transform3d(shooterPose.getTranslation(), shooterPose.getRotation()));
    }

    return currentFieldPose;
  }

  public static void update() {
    if (inShooter) {
      return;
    }

    Translation3d posDelta = fieldVelocity.times(Constants.NoteSim.dt);

    currentFieldPose =
        new Pose3d(
            currentFieldPose.getTranslation().plus(posDelta), currentFieldPose.getRotation());

    if (currentFieldPose.getX() <= -Constants.NoteSim.OUT_OF_FIELD_MARGIN
        || currentFieldPose.getX()
            >= Constants.NoteSim.FIELD_SIZE.getX() + Constants.NoteSim.OUT_OF_FIELD_MARGIN
        || currentFieldPose.getY() <= -Constants.NoteSim.OUT_OF_FIELD_MARGIN
        || currentFieldPose.getY()
            >= Constants.NoteSim.FIELD_SIZE.getY() + Constants.NoteSim.OUT_OF_FIELD_MARGIN
        || currentFieldPose.getZ() <= 0.0) {
      fieldVelocity = new Translation3d();
    } else {
      fieldVelocity =
          fieldVelocity.minus(Constants.NoteSim.GRAVITY_TRANSLATION3D.times(Constants.NoteSim.dt));
      double norm = fieldVelocity.getNorm();

      double fDrag =
          0.5
              * Constants.NoteSim.AIR_DENSITY
              * Math.pow(norm, 2)
              * Constants.NoteSim.DRAG_COEFFICIENT
              * Constants.NoteSim.CROSSECTION_AREA;
      double deltaV = (Constants.NoteSim.MASS * fDrag) * Constants.NoteSim.dt;

      double t = (norm - deltaV) / norm;
      fieldVelocity = fieldVelocity.times(t);
      noteTrajectory.add(currentFieldPose.getTranslation());
    }
  }

  public static void logNoteInfo() {
    Logger.recordOutput(
        "SimNoteTrajectory", NoteSimulator.getNoteTrajectory().toArray(new Translation3d[0]));
    Logger.recordOutput("SimNotePose3d", getFieldPose(Constants.NoteSim.SHOOTER_POSE3D));
  }
}
