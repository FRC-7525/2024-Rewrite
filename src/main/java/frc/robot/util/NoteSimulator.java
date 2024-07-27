package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class NoteSimulator {
    private static Drive drive;

    private static Pose3d currentFieldPose = new Pose3d();
    private static Translation3d fieldVelocity = new Translation3d();
    private static boolean inShooter = false;
    private static List<Translation3d> noteTrajectory = new ArrayList<>();

    private static final double AIR_DENSITY = 1.225;
    private static final double DRAG_COEFFICIENT = 0.45;
    private static final double CROSSECTION_AREA = 0.11;
    private static final double MASS = 0.235;

    private static final Pose3d shooterPose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final Translation2d fieldSize = new Translation2d(16.54, 8.21); //stolen from 3015 constants


    public static void setDrive(Drive drivesystem)
    {
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

        currentFieldPose = getFieldPose(shooterPose3d);
        inShooter = false;

        fieldVelocity = new Translation3d(velocity, currentFieldPose.getRotation());

        ChassisSpeeds robotVel = new ChassisSpeeds(0, 0, 0); //assumes robot is not moving
        ChassisSpeeds fieldRel =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                robotVel, drive.getRotation());

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

        double dt = 0.02;

        Translation3d posDelta = fieldVelocity.times(dt);

        currentFieldPose =
            new Pose3d(
                currentFieldPose.getTranslation().plus(posDelta), currentFieldPose.getRotation());

        if (currentFieldPose.getX() <= -0.25
            || currentFieldPose.getX() >= fieldSize.getX() + 0.25
            || currentFieldPose.getY() <= -0.25
            || currentFieldPose.getY() >= fieldSize.getY() + 0.25
            || currentFieldPose.getZ() <= 0.0) {
            fieldVelocity = new Translation3d();
        } else {
            fieldVelocity = fieldVelocity.minus(new Translation3d(0.0, 0.0, 9.81 * dt));
            double norm = fieldVelocity.getNorm();

            double fDrag = 0.5 * AIR_DENSITY * Math.pow(norm, 2) * DRAG_COEFFICIENT * CROSSECTION_AREA;
            double deltaV = (MASS * fDrag) * dt;

            double t = (norm - deltaV) / norm;
            fieldVelocity = fieldVelocity.times(t);
            noteTrajectory.add(currentFieldPose.getTranslation());
        }
    }

    public static void logNoteInfo() {
        Logger.recordOutput("SimNoteTrajectory", NoteSimulator.getNoteTrajectory().toArray(new Translation3d[0]));
        Logger.recordOutput("SimNotePose3d", getFieldPose(shooterPose3d));
    }
}
