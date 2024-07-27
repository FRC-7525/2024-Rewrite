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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Intake {
    // Motor setpoints for the intake motors.
    public static final double OFF = 0.0;
    public static final double REVERSE = -10;
    public static final double ON = 10;
    public static final double DOWN = Math.PI;
    public static final double GEARING = 67.5;
  }
  public static final class Shooter {
    public static final double SHOOTER_MAX = 0.1;
    public static final double SHOOTER_MAX_SPEED_DEVIATION = 5;
    //not sure if this is too high or low
    public static final double CIRCUMFRENCE_OF_SHOOTER_SPINNER = 4; //need to confirm with mech
  }

  public static final class AmpBar {
    public static final double PIVOT_GEARING = 0.5; // random val
    public static final double PIVOT_JKG_METERS_SQUARED = 2; // random val
    public static final double PIVOT_ARM_LENGTH = .378;
    public static final double PIVOT_MIN_ANGLE = 0;
    public static final double PIVOT_MAX_ANGLE = Units.degreesToRadians(114.1633329); // 293.... - 179
    public static final double PIVOT_STARTING_ANGLE = 0;

    public static final double SPINNER_GEARING = .5; //random val
    public static final double SPINNER_JKG_METERS_SQUARED = .5; //random val
  }

  public static final class NoteSim{
    public static final double AIR_DENSITY = 1.225;
    public static final double DRAG_COEFFICIENT = 0.45;
    public static final double CROSSECTION_AREA = 0.11;
    public static final double MASS = 0.235;

    public static final Pose3d SHOOTER_POSE3D = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final Translation2d FIELD_SIZE = new Translation2d(16.54, 8.21); //stolen from 3015 constants

    public static final double dt = 0.2; //change in time for note sim
    public static final Translation3d GRAVITY_TRANSLATION3D = new Translation3d(0, 0, 9.8);
    public static final double OUT_OF_FIELD_MARGIN = .025;
  }
}
