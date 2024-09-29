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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

	public static final double SLOW_UPDATE_FREQ = 50;

	// Conversion Factors
	public static final double RADIAN_CF = (Math.PI * 2);
	public static final double RPM_TO_RPS_CF = 60;
	public static final double DIAM_TO_RADIUS_CF = 2.0;
	public static final double AVG_TWO_ITEM_F = 2.0;

	public static final Mode currentMode = Mode.REAL;

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

		public static final PIDConstants REAL_IN_PID = new PIDConstants(0.08, 0, 0.001);
		public static final PIDConstants REAL_OUT_PID = new PIDConstants(0.25, 0, 0.002);

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

		// In DEGREES (Pivot setpoints)
		public static final double DOWN = -237;
		public static final double IN = 0;

		// In RPS (Spinner Setpoints)
		public static final double REVERSE = -3;
		public static final double ON = 3;

		// Erros of Margin
		public static final double WHEEL_ERROR_OF_MARGIN = 1;
		public static final double PIVOT_ERROR_OF_MARGIN = 10;

		// Beam Break
		public static final int BEAM_BREAK_PORT = 8;
		public static final double DEBOUNCE_TIME = 0.3;
	}

	public static final class Shooter {

		public static final double ERROR_OF_MARGIN = 1.0;

		// CAN IDs
		public static final int LEFT_SHOOTER_ID = 15;
		public static final int RIGHT_SHOOTER_ID = 14;

		// Shooter Setpoints (RPS)
		public static final double OFF = 0.0;
		public static final double FEEDING_AMP = 20;
		public static final double SHOOTING = 15.0;

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

		public static final double ERROR_OF_MARGIN = 3;

		// PID
		public static final PIDConstants SIM_PID = new PIDConstants(3, 0, 1.5);
		public static final PIDConstants REAL_PID = new PIDConstants(0.1, 0, 0);

		// Motor CAN IDs
		public static final int LEFT_PIVOT_ID = 30;
		public static final int RIGHT_PIVOT_ID = 31;
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
		public static final double SHOOTING = -10;
		public static final double FEEDING = -1;
		public static final double OFF = 0.0;

		// IN DEGREES (Pivot Setpoints)
		public static final double OUT = -87;
		public static final double FEEDING_POSITION = -79;
		public static final double IN = Units.degreesToRadians(0.0);

		// Beam Break
		public static final int BEAM_BREAK_PORT = 9;
		public static final double DEBOUNCE_TIME = 0.3;

		//State Transitions
		public static final double TIME_FOR_SCORING = 3;
	}

	public static final class Drive {

		public static final double DISCRETIZE_TIME_SECONDS = 0.02;
		public static final double CONTROLLER_DEADBAND = 0.05;
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

		// Auto Config
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(7.0, 0.0, 0.25);
		public static final PIDConstants ROTATION_PID = new PIDConstants(4.0, 0.0, 0.4);
		public static final double MAX_MODULE_SPEED = 6.0;

		// Configs
		public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
		public static final double MAX_LINEAR_SPEED = Units.feetToMeters(20);
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

		public static final class Navx2 {

			public static final double UPDATE_FREQUENCY = 1000.0;
		}

		public static final class Module {

			public static final double ODOMETRY_FREQUENCY = 250.0;

			public static final FFConstants REPLAY_FF = new FFConstants(0.1, 0.13);
			public static final PIDConstants REPLAY_DRIVE_PID = new PIDConstants(0.05, 0.0, 0.0);
			public static final PIDConstants REPLAY_TURN_PID = new PIDConstants(7.0, 0.0, 0.0);

			public static final FFConstants SIM_FF = new FFConstants(0.0, 0.13);
			public static final PIDConstants SIM_DRIVE_PID = new PIDConstants(0.1, 0.0, 0.0);
			public static final PIDConstants SIM_TURN_PID = new PIDConstants(10.0, 0.0, 0.0);

			// Hope this works??? This should be tuned using SYSID or Power, I, and Damping method
			public static final FFConstants REAL_FF = new FFConstants(0, 0);
			public static final PIDConstants REAL_DRIVE_PID = new PIDConstants(0.05, 0.0, 0.0);
			public static final PIDConstants REAL_TURN_PID = new PIDConstants(7.0, 0.0, 0.0);

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
				public static final double DRIVE_GEAR_RATIO = 5.357;
				// (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
				public static final double TURN_GEAR_RATIO = 21.4286;

				public static final double DRIVE_CURRENT_LIMIT = 40.0;
				public static final int TURN_CURRENT_LIMIT = 30;
				// Stuff
				public static final double TALON_UPDATE_FREQUENCY_HZ = 50.0;

				public static final int SPARK_TIMEOUT_MS = 250;
				public static final int SPARK_MEASURMENT_PERIOD_MS = 10;
				public static final int SPARK_AVG_DEPTH = 2;
				public static final double SPARK_FRAME_PERIOD = 1000.0 / ODOMETRY_FREQUENCY;

				// CAN/Device IDS and offsets (May be Wrong, guessed which ids are correct off vibes)

				// TODO: Confirm module IDs are associated with correct module and tune module offsets if
				// last years dont work

				// Front Left Module
				public static final int DRIVE0_ID = 2;
				public static final int TURN0_ID = 1;
				public static final int CANCODER0_ID = 3;
				public static final double OFFSET0 = Units.degreesToRadians(171.826171875);

				// Front Right Module
				public static final int DRIVE1_ID = 5;
				public static final int TURN1_ID = 4;
				public static final int CANCODER1_ID = 6;
				public static final double OFFSET1 = Units.degreesToRadians(-23.115234375);

				// Back Left Module
				public static final int DRIVE2_ID = 11;
				public static final int TURN2_ID = 10;
				public static final int CANCODER2_ID = 12;
				public static final double OFFSET2 = Units.degreesToRadians(220.517578125);

				// Back Right Module
				public static final int DRIVE3_ID = 8;
				public static final int TURN3_ID = 7;
				public static final int CANCODER3_ID = 9;
				public static final double OFFSET3 = Units.degreesToRadians(-67.939453125);
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

	public static final class Climber {

		public static final int LEFT_ID = 33;
		public static final int RIGHT_ID = 34;

		// IN ROTATIONS (climber rotates a lot ig)
		// TODO: TUNE
		public static final double ERROR_OF_MARGIN = 1;

		// TODO: Check if these are still right
		public static final double LEFT_CURRENT_LIMIT = 13;
		public static final double RIGHT_CURRENT_LIMIT = 13;

		// TODO: TUNE
		public static final PIDConstants REAL_PID = new PIDConstants(0.05, 0, 0);
		public static final PIDConstants SIM_PID = new PIDConstants(1, 0, 0);
	}

	public static final class AutoAlign {

		public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(3, 0, 0);
		public static final PIDConstants ROTATIONAL_PID = new PIDConstants(5, 0, 0);
		public static final double TIME_TO_FEED = 1.0; // seconds
		public static final double TIME_FOR_SCORING = .5; // seconds

		public static final Pose2d redAmpSpeakerPose = new Pose2d(
			15.59,
			6.644,
			new Rotation2d(Math.toRadians(120.5))
		);
		public static final Pose2d blueAmpSpeakerPose = new Pose2d(
			0.909,
			6.644,
			new Rotation2d(Math.toRadians(55.5))
		);
		public static final Pose2d redSourceSpeakerPose = new Pose2d(
			15.636,
			4.39,
			new Rotation2d(Math.toRadians(-122.5))
		);
		public static final Pose2d blueSourceSpeakerPose = new Pose2d(
			0.864,
			4.39,
			new Rotation2d(Math.toRadians(-62.5))
		);
		public static final Pose2d redAmpPose = new Pose2d(
			14.7,
			7.72,
			new Rotation2d(Math.toRadians(-90))
		);
		public static final Pose2d blueAmpPose = new Pose2d(
			1.85,
			7.72,
			new Rotation2d(Math.toRadians(-90))
		);

		public static final Pose2d blueSpeakerPose = new Pose2d(
			1.45,
			5.50,
			new Rotation2d(Math.toRadians(0))
		);
		public static final Pose2d redSpeakerPose = new Pose2d(
			15.1,
			5.50,
			new Rotation2d(Math.toRadians(180))
		);

		public static final double TRANSLATION_ERROR_MARGIN = 0.4;
		public static final double ROTATION_ERROR_MARGIN = 0.4;
	}

	public static final class NoteVision {

		public static final double CAMERA_HEIGHT = 22; // need to confirm
	}

	public static final class Vision {

		public static final double LAST_VISION_MEASURMENT_TIMER = 0.5;
		public static final boolean VISION_ENABLED = true;
		public static final double STD_TRUSTABLE_DISTANCE = 6;
		public static final Matrix<N3, N1> SINGLE_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds, if you only see one tag, ie less accuracy/trust so higher values bc we don't trust it
		public static final Matrix<N3, N1> MULTI_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds,  if you see multiple tags, ie more accuracy/trust so lower values bc we trust it
		public static final double[] TAG_WEIGHTS = {
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
		}; // how significantly important each tag is
	}
}
