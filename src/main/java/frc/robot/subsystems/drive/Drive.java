// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends Subsystem<DriveStates> {

  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final VisionIO visionIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();
  @AutoLogOutput public boolean useVision = true;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      VisionIO visionIO) {

    super("Drive", DriveStates.REGULAR_DRIVE);

    this.gyroIO = gyroIO;
    this.visionIO = visionIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Triggers
    addTrigger(
        DriveStates.REGULAR_DRIVE,
        DriveStates.SLOW_MODE,
        () -> Constants.controller.getRightBumper());
    addTrigger(
        DriveStates.REGULAR_DRIVE,
        DriveStates.SPEED_MAXXING,
        () -> Constants.controller.getLeftBumper());
  }

  @Override
  public void runState() {
    drive(
        this,
        () -> Constants.controller.getLeftY(),
        () -> Constants.controller.getLeftX(),
        () -> -Constants.controller.getRightX(),
        getState().getRotationModifier(),
        getState().getTranslationModifier());
  }

  public void drive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double rotationMultiplier,
      double translationMultiplier) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
            Constants.Drive.CONTROLLER_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Drive.CONTROLLER_DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field relative speeds & send command
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * translationMultiplier,
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * translationMultiplier,
            omega * drive.getMaxAngularSpeedRadPerSec() * rotationMultiplier,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  @Override
  public void periodic() {
    super.periodic();
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      // Vision jibberish (taken directly from Aembot 2024 repo)
      visionIO.updatePose(getPose());
      visionIO.updateInputs(visionInputs);
      Logger.processInputs("Drive/AprilTagVision", visionInputs);

      // Bounds check the pose is actually on the field, Deleted bc I dont want it
      // && Math.abs(visionInputs.visionPoses[j].getZ()) < 0.2
      // && visionInputs.visionPoses[j].getX() > 0
      // && visionInputs.visionPoses[j].getX() < 16.5
      // && visionInputs.visionPoses[j].getY() > 0
      // && visionInputs.visionPoses[j].getY() < 8.5
      // && visionInputs.visionPoses[j].getRotation().getX() < 0.2
      // && visionInputs.visionPoses[j].getRotation().getY() < 0.2)

      for (int j = 0; j < visionInputs.timestamps.length; j++) {
        if (visionInputs.timestamps[j] >= 1.0) {
          if (visionInputs.timestamps[j] > (Logger.getTimestamp() / 1.0e6)) {
            visionInputs.timestamps[j] = (Logger.getTimestamp() / 1.0e6) - visionInputs.latency[j];
          }
          Logger.recordOutput("Drive/AprilTagPose" + j, visionInputs.visionPoses[j].toPose2d());
          Logger.recordOutput(
              "Drive/AprilTagStdDevs" + j,
              Arrays.copyOfRange(visionInputs.visionStdDevs, 3 * j, 3 * j + 3));
          Logger.recordOutput("Drive/AprilTagTimestamps" + j, visionInputs.timestamps[j]);

          if (useVision) {
            poseEstimator.addVisionMeasurement(
                visionInputs.visionPoses[j].toPose2d(),
                visionInputs.timestamps[j],
                VecBuilder.fill(
                    visionInputs.visionStdDevs[3 * j],
                    visionInputs.visionStdDevs[3 * j + 1],
                    visionInputs.visionStdDevs[3 * j + 2]));
          }
        } else {
          Logger.recordOutput("Drive/AprilTagPose" + j, new Pose2d());
          Logger.recordOutput("Drive/AprilTagStdDevs" + j, new double[] {0.0, 0.0, 0.0});
        }
      }
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }


	static final Lock odometryLock = new ReentrantLock();
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules = new Module[Constants.Drive.NUM_MODULES]; // FL, FR, BL, BR
	private PPDriveWrapper autoConfig;

	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
	private Rotation2d rawGyroRotation = new Rotation2d();
	private SwerveModulePosition[] lastModulePositions = // For delta tracking
		new SwerveModulePosition[] {
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
		};
	private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
		kinematics,
		rawGyroRotation,
		lastModulePositions,
		new Pose2d()
	);

	public Drive(
		GyroIO gyroIO,
		ModuleIO flModuleIO,
		ModuleIO frModuleIO,
		ModuleIO blModuleIO,
		ModuleIO brModuleIO
	) {
		super("Drive", DriveStates.REGULAR_DRIVE);
		autoConfig = new PPDriveWrapper(this);

		this.gyroIO = gyroIO;
		modules[0] = new Module(flModuleIO, 0);
		modules[1] = new Module(frModuleIO, 1);
		modules[2] = new Module(blModuleIO, 2);
		modules[3] = new Module(brModuleIO, 3);

		// Start threads (no-op for each if no signals have been created)
		PhoenixOdometryThread.getInstance().start();
		SparkMaxOdometryThread.getInstance().start();

		// Triggers
		addTrigger(DriveStates.REGULAR_DRIVE, DriveStates.SLOW_MODE, () ->
			Constants.controller.getRightBumper()
		);
		addTrigger(DriveStates.REGULAR_DRIVE, DriveStates.SPEED_MAXXING, () ->
			Constants.controller.getLeftBumper()
		);

		// Back to Off
		addTrigger(DriveStates.SPEED_MAXXING, DriveStates.SLOW_MODE, () ->
			Constants.controller.getLeftBumperReleased()
		);
		addTrigger(DriveStates.SLOW_MODE, DriveStates.REGULAR_DRIVE, () ->
			Constants.controller.getRightBumperReleased()
		);
	}

	@Override
	public void runState() {
		// Can't run in auto otherwise it will constantly tell drive not to drive in auto (and thats not good)
		if (DriverStation.isTeleop()) {
			drive(
				this,
				() -> Constants.controller.getLeftY(),
				() -> Constants.controller.getLeftX(),
				() -> -Constants.controller.getRightX(),
				getState().getRotationModifier(),
				getState().getTranslationModifier()
			);
		}
	}

	public void drive(
		Drive drive,
		DoubleSupplier xSupplier,
		DoubleSupplier ySupplier,
		DoubleSupplier omegaSupplier,
		double rotationMultiplier,
		double translationMultiplier
	) {
		// Apply deadband
		double linearMagnitude = MathUtil.applyDeadband(
			Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
			Constants.Drive.CONTROLLER_DEADBAND
		);
		Rotation2d linearDirection = new Rotation2d(
			xSupplier.getAsDouble(),
			ySupplier.getAsDouble()
		);
		double omega = MathUtil.applyDeadband(
			omegaSupplier.getAsDouble(),
			Constants.Drive.CONTROLLER_DEADBAND
		);

		// Square values
		linearMagnitude = linearMagnitude * linearMagnitude;
		omega = Math.copySign(omega * omega, omega);

		// Calcaulate new linear velocity
		Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
			.transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
			.getTranslation();

		// Convert to field relative speeds & send command
		boolean isFlipped =
			DriverStation.getAlliance().isPresent() &&
			DriverStation.getAlliance().get() == Alliance.Red;
		drive.runVelocity(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				linearVelocity.getX() *
				drive.getMaxLinearSpeedMetersPerSec() *
				translationMultiplier,
				linearVelocity.getY() *
				drive.getMaxLinearSpeedMetersPerSec() *
				translationMultiplier,
				omega * drive.getMaxAngularSpeedRadPerSec() * rotationMultiplier,
				isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()
			)
		);
	}

	@Override
	public void periodic() {
		super.periodic();
		odometryLock.lock(); // Prevents odometry updates while reading data
		gyroIO.updateInputs(gyroInputs);
		for (var module : modules) {
			module.updateInputs();
		}
		odometryLock.unlock();
		Logger.processInputs("Drive/Gyro", gyroInputs);
		for (var module : modules) {
			module.periodic();
		}

		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
		}
		// Log empty setpoint states when disabled
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
		}

		// Update odometry
		double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++) {
			// Read wheel positions and deltas from each module
			SwerveModulePosition[] modulePositions =
				new SwerveModulePosition[Constants.Drive.NUM_MODULES];
			SwerveModulePosition[] moduleDeltas =
				new SwerveModulePosition[Constants.Drive.NUM_MODULES];
			for (int moduleIndex = 0; moduleIndex < Constants.Drive.NUM_MODULES; moduleIndex++) {
				modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
				moduleDeltas[moduleIndex] = new SwerveModulePosition(
					modulePositions[moduleIndex].distanceMeters -
					lastModulePositions[moduleIndex].distanceMeters,
					modulePositions[moduleIndex].angle
				);
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}

			// Update gyro angle
			if (gyroInputs.connected) {
				// Use the real gyro angle
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				// Use the angle delta from the kinematics and module deltas
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

			// Apply update
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
		}
	}

	/**
	 * Runs the drive at the desired velocity.
	 *
	 * @param speeds Speeds in meters/sec
	 */
	public void runVelocity(ChassisSpeeds speeds) {
		// Calculate module setpoints
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(
			speeds,
			Constants.Drive.DISCRETIZE_TIME_SECONDS
		);
		SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			setpointStates,
			Constants.Drive.MAX_LINEAR_SPEED
		);

		// Send setpoints to modules
		SwerveModuleState[] optimizedSetpointStates =
			new SwerveModuleState[Constants.Drive.NUM_MODULES];
		for (int i = 0; i < Constants.Drive.NUM_MODULES; i++) {
			// The module returns the optimized state, useful for logging
			optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
		}

		// Log setpoint states
		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
	}

	/** Stops the drive. */
	public void stop() {
		runVelocity(new ChassisSpeeds());
	}

	/**
	 * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
	 * return to their normal orientations the next time a nonzero velocity is requested.
	 */
	public void stopWithX() {
		Rotation2d[] headings = new Rotation2d[Constants.Drive.NUM_MODULES];
		for (int i = 0; i < Constants.Drive.NUM_MODULES; i++) {
			headings[i] = getModuleTranslations()[i].getAngle();
		}
		kinematics.resetHeadings(headings);
		stop();
	}

	/** Returns the module states (turn angles and drive velocities) for all of the modules. */
	@AutoLogOutput(key = "SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[Constants.Drive.NUM_MODULES];
		for (int i = 0; i < Constants.Drive.NUM_MODULES; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	/** Returns the module positions (turn angles and drive positions) for all of the modules. */
	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] states = new SwerveModulePosition[Constants.Drive.NUM_MODULES];
		for (int i = 0; i < Constants.Drive.NUM_MODULES; i++) {
			states[i] = modules[i].getPosition();
		}
		return states;
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public ChassisSpeeds getChassisSpeed() {
		ChassisSpeeds robotChassisSpeed = kinematics.toChassisSpeeds(getModuleStates());
		return robotChassisSpeed;
	}

	public double calculateVelocity() {
		double robotSpeed = Math.sqrt(
			Math.pow(getChassisSpeed().vxMetersPerSecond, 2) +
			Math.pow(getChassisSpeed().vyMetersPerSecond, 2)
		);
		return robotSpeed;
	}

	/** Returns the current odometry rotation. */
	public Rotation2d getRotation() {
		return getPose().getRotation();
	}

	/** Resets the current odometry pose. */
	public void setPose(Pose2d pose) {
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	/**
	 * Adds a vision measurement to the pose estimator.
	 *
	 * @param visionPose The pose of the robot as measured by the vision camera.
	 * @param timestamp The timestamp of the vision measurement in seconds.
	 */
	public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
		poseEstimator.addVisionMeasurement(visionPose, timestamp);
	}

	/** Returns the maximum linear speed in meters per sec. */
	public double getMaxLinearSpeedMetersPerSec() {
		return Constants.Drive.MAX_LINEAR_SPEED;
	}

	/** Returns the maximum angular speed in radians per sec. */
	public double getMaxAngularSpeedRadPerSec() {
		return Constants.Drive.MAX_ANGULAR_SPEED;
	}

	/** Returns an array of module translations. */
	public static Translation2d[] getModuleTranslations() {
		return new Translation2d[] {
			new Translation2d(
				Constants.Drive.TRACK_WIDTH_X / Constants.DIAM_TO_RADIUS_CF,
				Constants.Drive.TRACK_WIDTH_Y / Constants.DIAM_TO_RADIUS_CF
			),
			new Translation2d(
				Constants.Drive.TRACK_WIDTH_X / Constants.DIAM_TO_RADIUS_CF,
				-Constants.Drive.TRACK_WIDTH_Y / Constants.DIAM_TO_RADIUS_CF
			),
			new Translation2d(
				-Constants.Drive.TRACK_WIDTH_X / Constants.DIAM_TO_RADIUS_CF,
				Constants.Drive.TRACK_WIDTH_Y / Constants.DIAM_TO_RADIUS_CF
			),
			new Translation2d(
				-Constants.Drive.TRACK_WIDTH_X / Constants.DIAM_TO_RADIUS_CF,
				-Constants.Drive.TRACK_WIDTH_Y / Constants.DIAM_TO_RADIUS_CF
			),
		};
	}
  
}
