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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOOutputs;
import org.littletonrobotics.junction.Logger;

public class Module {

	private static final double WHEEL_RADIUS = Constants.Drive.WHEEL_RADIUS;

	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private final ModuleIOOutputs outputs;
	private final int index;

	private final SimpleMotorFeedforward driveFeedforward;
	private final PIDController driveFeedback;
	private final PIDController turnFeedback;
	private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
	private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
	private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

	private SwerveModuleState lastModuleState;

	public Module(ModuleIO io, int index) {
		this.io = io;
		this.index = index;

		outputs = new ModuleIOOutputs();

		// Switch constants based on mode (the physics simulator is treated as a
		// separate robot with different tuning)
		switch (Constants.currentMode) {
			// TODO: Leaving this as a warning incase I did something bad, before "REAL"
			// didnt assign
			// any PID/FF values for controllers idk why
			case REAL:
				// TODO: TEST THIS
				driveFeedforward = createDriveFeedforward(
					Constants.Drive.OPTIMAL_VOLTAGE,
					Constants.Drive.MAX_LINEAR_SPEED,
					Constants.Drive.WHEEL_GRIP_COEFFICIENT_OF_FRICTION
				);
				driveFeedback = new PIDController(
					Constants.Drive.Module.REAL_DRIVE_PID.kP,
					Constants.Drive.Module.REAL_DRIVE_PID.kI,
					Constants.Drive.Module.REAL_DRIVE_PID.kD
				);
				turnFeedback = new PIDController(
					Constants.Drive.Module.REAL_TURN_PID.kP,
					Constants.Drive.Module.REAL_TURN_PID.kI,
					Constants.Drive.Module.REAL_TURN_PID.kD
				);
				break;
			case REPLAY:
				driveFeedforward = new SimpleMotorFeedforward(
					Constants.Drive.Module.REPLAY_FF.kS,
					Constants.Drive.Module.REPLAY_FF.kV
				);
				driveFeedback = new PIDController(
					Constants.Drive.Module.REPLAY_DRIVE_PID.kP,
					Constants.Drive.Module.REPLAY_DRIVE_PID.kI,
					Constants.Drive.Module.REPLAY_DRIVE_PID.kD
				);
				turnFeedback = new PIDController(
					Constants.Drive.Module.REPLAY_TURN_PID.kP,
					Constants.Drive.Module.REPLAY_TURN_PID.kI,
					Constants.Drive.Module.REPLAY_TURN_PID.kD
				);
				break;
			case SIM:
				driveFeedforward = new SimpleMotorFeedforward(
					Constants.Drive.Module.SIM_FF.kS,
					Constants.Drive.Module.SIM_FF.kV
				);
				driveFeedback = new PIDController(
					Constants.Drive.Module.SIM_DRIVE_PID.kP,
					Constants.Drive.Module.SIM_DRIVE_PID.kI,
					Constants.Drive.Module.SIM_DRIVE_PID.kD
				);
				turnFeedback = new PIDController(
					Constants.Drive.Module.SIM_TURN_PID.kP,
					Constants.Drive.Module.SIM_TURN_PID.kI,
					Constants.Drive.Module.SIM_TURN_PID.kD
				);
				break;
			default:
				driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
				driveFeedback = new PIDController(0.0, 0.0, 0.0);
				turnFeedback = new PIDController(0.0, 0.0, 0.0);
				break;
		}

		turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
		setBrakeMode(true);
	}

	// REAL LIFE ONLY!!!!!!!!

	public static SimpleMotorFeedforward createDriveFeedforward(
		double optimalVoltage,
		double maxSpeed,
		double wheelGripCoefficientOfFriction
	) {
		double kv = optimalVoltage / maxSpeed;
		/// ^ Volt-seconds per meter (max voltage divided by max speed)
		double ka = optimalVoltage / calculateMaxAcceleration(wheelGripCoefficientOfFriction);
		/// ^ Volt-seconds^2 per meter (max voltage divided by max accel)
		return new SimpleMotorFeedforward(0, kv, ka);
	}

	// 9.81 is "gravity"
	public static double calculateMaxAcceleration(double cof) {
		return cof * Constants.GRAVITY;
	}

	/**
	 * Update inputs without running the rest of the periodic logic. This is useful
	 * since these
	 * updates need to be properly thread-locked.
	 */
	public void updateInputs() {
		io.updateInputs(inputs);
	}

	public void updateOutputs() {
		io.updateOutputs(outputs);
	}

	public void periodic() {
		Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
		Logger.recordOutput(
			"Drive/Module" + Integer.toString(index) + "/" + "AbsoluteEncoderPositionAsDouble",
			inputs.turnAbsolutePosition.getDegrees()
		);

		updateOutputs();
		Logger.recordOutput(
			"Drive/Module" + Integer.toString(index) + "/DriveAppliedVolts",
			outputs.driveAppliedVolts
		);
		Logger.recordOutput(
			"Drive/Module" + Integer.toString(index) + "/TurnAppliedVolts",
			outputs.turnAppliedVolts
		);

		// On first cycle, reset relative turn encoder
		// Wait until absolute angle is nonzero in case it wasn't initialized yet
		if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
			turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
		}

		// Run closed loop turn control
		if (angleSetpoint != null) {
			io.setTurnVoltage(
				turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians())
			);

			// Run closed loop drive control
			// Only allowed if closed loop turn control is running
			if (speedSetpoint != null) {
				// Scale velocity based on turn error
				//
				// When the error is 90°, the velocity setpoint should be 0. As the wheel turns
				// towards the setpoint, its velocity should increase. This is achieved by
				// taking the component of the velocity in the direction of the setpoint.
				double adjustSpeedSetpoint =
					speedSetpoint * Math.cos(turnFeedback.getPositionError());

				// Run drive controller
				double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
				io.setDriveVoltage(
					driveFeedforward.calculate(velocityRadPerSec) +
					driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec)
				);
			}
		}

		// Calculate positions for odometry
		int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS;
			Rotation2d angle =
				inputs.odometryTurnPositions[i].plus(
						turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d()
					);
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}
	}

	// TODO: TEST
	public static void antiJitter(
		SwerveModuleState moduleState,
		SwerveModuleState lastModuleState,
		double maxSpeed
	) {
		if (
			Math.abs(moduleState.speedMetersPerSecond) <=
			(maxSpeed * Constants.Drive.ANTI_JITTER_DRIVE_THRESHOLD)
		) {
			moduleState.angle = lastModuleState.angle;
		}
	}

	/**
	 * Runs the module with the specified setpoint state. Returns the optimized
	 * state.
	 */
	public SwerveModuleState runSetpoint(SwerveModuleState state) {
		// Optimize state based on current angle
		// Controllers run in "periodic" when the setpoint is not null

		// Set last moule state at start
		if (lastModuleState == null) {
			lastModuleState = state;
		}

		var optimizedState = SwerveModuleState.optimize(state, getAngle());
		antiJitter(state, lastModuleState, Constants.Drive.MAX_LINEAR_SPEED);

		// Update setpoints, controllers run in "periodic"
		angleSetpoint = optimizedState.angle;
		speedSetpoint = optimizedState.speedMetersPerSecond;

		lastModuleState = state;
		return optimizedState;
	}

	/** Disables all outputs to motors. */
	public void stop() {
		io.setTurnVoltage(0.0);
		io.setDriveVoltage(0.0);

		// Disable closed loop control for turn and drive
		angleSetpoint = null;
		speedSetpoint = null;
	}

	/** Sets whether brake mode is enabled. */
	public void setBrakeMode(boolean enabled) {
		io.setDriveBrakeMode(enabled);
		io.setTurnBrakeMode(enabled);
	}

	/** Returns the current turn angle of the module. */
	public Rotation2d getAngle() {
		if (turnRelativeOffset == null) {
			return new Rotation2d();
		} else {
			return inputs.turnPosition.plus(turnRelativeOffset);
		}
	}

	/** Returns the current drive position of the module in meters. */
	public double getPositionMeters() {
		return inputs.drivePositionRad * WHEEL_RADIUS;
	}

	/** Returns the current drive velocity of the module in meters per second. */
	public double getVelocityMetersPerSec() {
		return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
	}

	/** Returns the module position (turn angle and drive position). */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	/** Returns the module state (turn angle and drive velocity). */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	/** Returns the module positions received this cycle. */
	public SwerveModulePosition[] getOdometryPositions() {
		return odometryPositions;
	}

	/** Returns the timestamps of the samples received this cycle. */
	public double[] getOdometryTimestamps() {
		return inputs.odometryTimestamps;
	}
}
