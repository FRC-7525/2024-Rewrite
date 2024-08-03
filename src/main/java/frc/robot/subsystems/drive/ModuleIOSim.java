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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {

	private DCMotorSim driveSim = new DCMotorSim(
		DCMotor.getNEO(Constants.Drive.Module.NUM_DRIVE_MOTORS),
		Constants.Drive.Module.Sim.DRIVE_GEARING,
		Constants.Drive.Module.Sim.DRIVE_MOI
	);
	private DCMotorSim turnSim = new DCMotorSim(
		DCMotor.getNEO(Constants.Drive.Module.NUM_TURN_MOTORS),
		Constants.Drive.Module.Sim.TURN_GEARING,
		Constants.Drive.Module.Sim.TURN_MOI
	);

	private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(
		Math.random() * Constants.RADIAN_CF
	);
	private double driveAppliedVolts = 0.0;
	private double turnAppliedVolts = 0.0;

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		driveSim.update(Constants.Drive.Module.Sim.LOOP_PERIOD_SECS);
		turnSim.update(Constants.Drive.Module.Sim.LOOP_PERIOD_SECS);

		inputs.drivePositionRad = driveSim.getAngularPositionRad();
		inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
		inputs.driveAppliedVolts = driveAppliedVolts;
		inputs.driveCurrentAmps = new double[] { Math.abs(driveSim.getCurrentDrawAmps()) };

		inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad()).plus(
			turnAbsoluteInitPosition
		);
		inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
		inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
		inputs.turnAppliedVolts = turnAppliedVolts;
		inputs.turnCurrentAmps = new double[] { Math.abs(turnSim.getCurrentDrawAmps()) };

		inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
		inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
		inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
	}

	@Override
	public void setDriveVoltage(double volts) {
		// :( it doesent work if you clamp volts regularly idk. It really just stopped working
		driveAppliedVolts = MathUtil.clamp(volts, Constants.MIN_VOLTS, Constants.MAX_VOLTS);
		driveSim.setInputVoltage(volts);
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnAppliedVolts = MathUtil.clamp(volts, Constants.MIN_VOLTS, Constants.MAX_VOLTS);
		turnSim.setInputVoltage(turnAppliedVolts);
	}
}
