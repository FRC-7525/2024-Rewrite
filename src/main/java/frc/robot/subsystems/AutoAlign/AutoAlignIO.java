package frc.robot.subsystems.AutoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveStates;
import frc.robot.subsystems.noteVision.NoteVision;

public class AutoAlignIO {

	private NoteVision noteVision;
	Drive driveSubsystem;
	PIDController translationalPIDController;
	PIDController rotationalPIDController;

	Pose2d targetPose2d;
	double appliedX;
	double appliedY;
	double appliedRotational;

	public AutoAlignIO(Drive driveSubsystem) {
		this.driveSubsystem = driveSubsystem;

		translationalPIDController = new PIDController(0, 0, 0);
		rotationalPIDController = new PIDController(0, 0, 0);

		targetPose2d = new Pose2d();
		noteVision = new NoteVision();
	}

	/* Returns if the robot is near the target pose */
	public boolean nearTargetPoint() {
		Pose2d currentPose2d = driveSubsystem.getPose();

		return (
			Math.abs(currentPose2d.getX() - targetPose2d.getX()) <
				Constants.AutoAlign.TRANSLATION_ERROR_MARGIN &&
			Math.abs(currentPose2d.getY() - targetPose2d.getY()) <
			Constants.AutoAlign.TRANSLATION_ERROR_MARGIN &&
			Math.abs(
				currentPose2d.getRotation().getDegrees() - targetPose2d.getRotation().getDegrees()
			) <
			Constants.AutoAlign.ROTATION_ERROR_MARGIN
		);
	}

	/*drives to target pose*/
	public void driveToTargetPose() {
		Pose2d currentPose2d = driveSubsystem.getPose();

		/*uses run velocity from drive and PID controllers to go to target pose */
		appliedX = translationalPIDController.calculate(currentPose2d.getX(), targetPose2d.getX());
		appliedY = translationalPIDController.calculate(currentPose2d.getY(), targetPose2d.getY());
		appliedRotational = rotationalPIDController.calculate(
			currentPose2d.getRotation().getRadians(),
			targetPose2d.getRotation().getRadians()
		);
		driveSubsystem.runVelocity(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				appliedX,
				appliedY,
				appliedRotational,
				driveSubsystem.getRotation()
			)
		);
	}

	public void driveToNotePose() {
		Pose2d currentPose2d = driveSubsystem.getPose();
		Pose2d visionPose = noteVision.getNotePose(currentPose2d);

		if (visionPose != null) {
			targetPose2d = visionPose; // Sets target pose to (best) note seen rather than whatever it is set to
		}

		/*uses run velocity from drive and PID controllers to go to target pose */
		appliedX = translationalPIDController.calculate(currentPose2d.getX(), targetPose2d.getX());
		appliedY = translationalPIDController.calculate(currentPose2d.getY(), targetPose2d.getY());
		appliedRotational = rotationalPIDController.calculate(
			currentPose2d.getRotation().getRadians(),
			targetPose2d.getRotation().getRadians()
		);
		driveSubsystem.runVelocity(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				appliedX,
				appliedY,
				appliedRotational,
				driveSubsystem.getRotation()
			)
		);
	}

	/* sets drive states */
	public void returnDriveToNormal() {
		driveSubsystem.setState(DriveStates.REGULAR_DRIVE);
	}

	public void lockDrive() {
		driveSubsystem.setState(DriveStates.AUTO_ALIGN);
	}

	/* returns target pose2d*/
	public Pose2d getTargetPose2d() {
		return targetPose2d;
	}

	/* return applied values */
	public double getAppliedX() {
		return appliedX;
	}

	public double getAppliedY() {
		return appliedY;
	}

	public double getAppliedRotational() {
		return appliedRotational;
	}

	/* Sets target pose that the robot will auto align to*/
	public void setTargetPose(Pose2d target) {
		targetPose2d = target;
	}

	/*changes PID values */
	public void configureTranslationalPID(double kP, double kI, double kD) {
		translationalPIDController.setPID(kP, kI, kD);
	}

	public void configurerotationalPID(double kP, double kI, double kD) {
		rotationalPIDController.setPID(kP, kI, kD);
	}

	public Pose2d getPose() {
		return driveSubsystem.getPose();
	}
}
