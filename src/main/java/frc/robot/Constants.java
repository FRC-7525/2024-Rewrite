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

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.FFConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final double SIM_UPDATE_TIME = 0.05;

	// Conversion Factors
	public static final double RADIAN_CF = (Math.PI * 2);
	public static final double RPM_TO_RPS_CF = 60;
	public static final double DIAM_TO_RADIUS_CF = 2.0;
	public static final double AVG_TWO_ITEM_F = 2.0;

	public static final Mode currentMode = Mode.SIM;

	public static final double MAX_VOLTS = 12.0;
	public static final double MIN_VOLTS = -12.0;

	public static final XboxController controller = new XboxController(0);
	public static final XboxController operatorController = new XboxController(1);

	public static enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY,
	}

	public static final class Intake {

		public static final Translation3d ZEROED_PIVOT_TRANSLATION = new Translation3d(
			0.31,
			0,
			0.24
		);

		// CAN IDs
		public static final int PIVOT_ID = 32;
		public static final int SPINNER_ID = 20;

		// PID
		public static final PIDConstants SIM_IN_PID = new PIDConstants(1, 0, 0);
		public static final PIDConstants SIM_OUT_PID = new PIDConstants(1, 0, 0);

		public static final PIDConstants REAL_IN_PID = new PIDConstants(1, 0, 0);
		public static final PIDConstants REAL_OUT_PID = new PIDConstants(1, 0, 0);

		// Sim Configs

		// Pivot
		public static final int NUM_PIVOT_MOTORS = 1;
		public static final double PIVOT_GEARING = 67.5;
		public static final double MAX_PIVOT_POSITION = Units.degreesToRadians(180.0);
		public static final double PIVOT_MOI = 0.192383865;
		public static final double PIVOT_LENGTH_METERS = 0.3;

		// Spinner
		public static final int NUM_SPINNER_MOTORS = 1;
		public static final double SPINNER_MOI = 0.01;
		public static final double SPINNER_GEARING = 1.0;
		public static final double OFF = 0.0;

		// In Rads (Pivot setpoints)
		public static final double DOWN = Units.degreesToRadians(180.0);
		public static final double IN = Units.degreesToRadians(0.0);

		// In RPS (Spinner Setpoints)
		public static final double REVERSE = -10.0;
		public static final double ON = 10.0;
	}

	public static final class Shooter {

		public static final double ERROR_OF_MARGIN = 2.0;

		// PID
		public static final PIDConstants SIM_PID = new PIDConstants(1, 0, 0);
		public static final PIDConstants REAL_PID = new PIDConstants(1, 0, 0);

		// CAN IDs
		public static final int LEFT_SHOOTER_ID = 15;
		public static final int RIGHT_SHOOTER_ID = 14;

		// Shooter Setpoints (RPS)
		public static final double OFF = 0.0;
		public static final double FEEDING_AMP = 25.0;
		public static final double SHOOTING = 50.0;

		// Sim Configs
		public static final int NUM_MOTORS = 2;
		public static final double SHOOTER_GEARING = 1.5;
		public static final double SHOOTER_MOI = 0.004;

		// spinner circumfrence need to check with mech
		public static final double CIRCUMFRENCE_OF_SHOOTER_SPINNER = 4;
	}

	public static final class AmpBar {

		public static final Translation3d ZEROED_PIVOT_TRANSLATION = new Translation3d(
			-0.317,
			0,
			0.525
		);

		public static final double ERROR_OF_MARGIN = 0.1;

		// PID
		public static final PIDConstants SIM_PID = new PIDConstants(3, 0, 1.5);
		public static final PIDConstants REAL_PID = new PIDConstants(1, 0, 0);

		// Motor CAN IDs
		public static final int LEFT_PIVOT_ID = 31;
		public static final int RIGHT_PIVOT_ID = 30;
		public static final int SPINNER_ID = 38;

		// Sim Configs

		// Spinner
		public static final double SPINNER_GEARING = 0.5;
		public static final double SPINNER_MOI = 0.5;
		public static final int NUM_SPINNER_MOTORS = 1;

		// Pivot
		public static final int NUM_PIVOT_MOTORS = 2;
		public static final double PIVOT_GEARING = 0.05;
		public static final double PIVOT_MOI = 0.05;
		public static final double PIVOT_LENGTH_METERS = 0.378;
		public static final double MIN_PIVOT_POSITION = -Units.degreesToRadians(114.163329);
		public static final double MAX_PIVOT_POSITION = Units.degreesToRadians(0);

		// Pivot and Spinner Setpoints

		// In RPS (Spinner Setpoints)
		public static final double SHOOTING = -0.5;
		public static final double FEEDING = -0.1;
		public static final double OFF = 0.0;

		// IN Rads (Pivot Setpoints)
		public static final double OUT = -Units.degreesToRadians(100.0);
		public static final double FEEDING_POSITION = -Units.degreesToRadians(93.0);
		public static final double IN = Units.degreesToRadians(0.0);
	}

	public static final class Drive {

		public static final double DISCRETIZE_TIME_SECONDS = 0.02;
		public static final double CONTROLLER_DEADBAND = 0.1;
		public static final int NUM_MODULES = 4;

		/* Rotation and Translation Modifers
    rm = rotation multiplier
    tm = translation multipliers
    aa = auto align
    */
		public static final double REGULAR_RM = 1.0;
		public static final double REGULAR_TM = 1.0;
		public static final double SLOW_RM = 0.5;
		public static final double SLOW_TM = 0.2;
		public static final double AA_RM = 0.8;
		public static final double AA_TM = 0.8;
		public static final double FAST_RM = 1.5;
		public static final double FAST_TM = 2.0;

		// Configs
		public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
		public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
		public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
		public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
		public static final double DRIVE_BASE_RADIUS = Math.hypot(
			TRACK_WIDTH_X / 2.0,
			TRACK_WIDTH_Y / 2.0
		);
		public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

		public static final class Pidgeon2 {

			public static final int DEVICE_ID = 20;
			public static final double UPDATE_FREQUENCY = 100.0;
		}

		public static final class Module {

			public static final double ODOMETRY_FREQUENCY = 250.0;

			// There isnt one for real its just all 0 so idk whats good with that
			public static final FFConstants REPLAY_FF = new FFConstants(0.1, 0.13);
			public static final PIDConstants REPLAY_DRIVE_PID = new PIDConstants(0.05, 0.0, 0.0);
			public static final PIDConstants REPLAY_TURN_PID = new PIDConstants(7.0, 0.0, 0.0);

			public static final FFConstants SIM_FF = new FFConstants(0.0, 0.13);
			public static final PIDConstants SIM_DRIVE_PID = new PIDConstants(0.1, 0.0, 0.0);
			public static final PIDConstants SIM_TURN_PID = new PIDConstants(10.0, 0.0, 0.0);

			public static final int NUM_TURN_MOTORS = 1;
			public static final int NUM_DRIVE_MOTORS = 1;

			public static final class Sim {

				public static final double LOOP_PERIOD_SECS = 0.02;

				// Configs
				public static final double DRIVE_GEARING = 6.75;
				public static final double DRIVE_MOI = 0.025;

				public static final double TURN_GEARING = 150.0 / 7.0;
				public static final double TURN_MOI = 0.004;
			}

			// TODO: Put constants from those abstractions in here
			public static final class SparkMax {}

			public static final class TalonFX {}

			public static final class Hybrid {

				// These are for l2 Mk4i mods, should be L3 plus
				public static final double DRIVE_GEAR_RATIO =
					(50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
				public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

				public static final double DRIVE_CURRENT_LIMIT = 40.0;
				public static final int TURN_CURRENT_LIMIT = 30;
				// Stuff
				public static final double TALON_UPDATE_FREQUENCY_HZ = 50.0;

				public static final int SPARK_TIMEOUT_MS = 250;
				public static final int SPARK_MEASURMENT_PERIOD_MS = 10;
				public static final int SPARK_AVG_DEPTH = 2;
				public static final double SPARK_FRAME_PERIOD = 1000.0 / ODOMETRY_FREQUENCY;

				// CAN/Device IDS and offsets
				public static final int DRIVE0_ID = 0;
				public static final int TURN0_ID = 1;
				public static final int CANCODER0_ID = 2;
				public static final double OFFSET0 = 0.0;

				public static final int DRIVE1_ID = 0;
				public static final int TURN1_ID = 1;
				public static final int CANCODER1_ID = 2;
				public static final double OFFSET1 = 0.0;

				public static final int DRIVE2_ID = 0;
				public static final int TURN2_ID = 1;
				public static final int CANCODER2_ID = 2;
				public static final double OFFSET2 = 0.0;

				public static final int DRIVE3_ID = 0;
				public static final int TURN3_ID = 1;
				public static final int CANCODER3_ID = 2;
				public static final double OFFSET3 = 0.0;
			}
		}

		public static final class OdoThread {

			public static final class Phoenix {

				public static final int QUE_CAPACITY = 20;
				public static final double SLEEP_TIME = 1000.0;
			}

			public static final class SparkMax {

				public static final int QUE_CAPACITY = 20;
				public static final double PERIOD = 1.0;
			}
		}
	}

	public static final class NoteSim {

		public static final double AIR_DENSITY = 1.225;
		public static final double DRAG_COEFFICIENT = 0.45;
		public static final double CROSSECTION_AREA = 0.11;
		public static final double MASS = 0.235;

		public static final Pose3d SHOOTER_POSE3D = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
		public static final Translation2d FIELD_SIZE = new Translation2d(16.54, 8.21); // stolen from 3015 constants

		public static final double dt = 0.2; // change in time for note sim
		public static final Translation3d GRAVITY_TRANSLATION3D = new Translation3d(0, 0, 9.8);
		public static final double OUT_OF_FIELD_MARGIN = .025;
	}
}
