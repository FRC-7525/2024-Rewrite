package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PPDriveWrapper extends SubsystemBase {
  Drive drive;

  PPDriveWrapper(Drive drive) {
    this.drive = drive;
    pathPlannerInit();
  }

  private Pose2d getPose() {
    return drive.getPose();
  }

  private void setPose(Pose2d pose) {
    drive.setPose(pose);
  }

  private ChassisSpeeds getChassisSpeed() {
    return drive.getChassisSpeed();
  }

  private void runVelocity(ChassisSpeeds chassisSpeeds) {
    drive.runVelocity(chassisSpeeds);
  }

  public void periodic() {}

  public void pathPlannerInit() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig(
            // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            Constants.Drive.TRANSLATION_PID, // Translation PID constants
            Constants.Drive.ROTATION_PID, // Rotation PID constants
            Constants.Drive.MAX_MODULE_SPEED, // Max module speed, in m/s
            Constants.Drive
                .DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to
            // furthest module.
            // Replans path if vision or odo detects errors (S tier)
            new ReplanningConfig(
                true, true) // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }
}
