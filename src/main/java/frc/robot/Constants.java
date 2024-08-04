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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

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
  public static final double RADIAN_CF = (Math.PI * 2);

  public static final Mode currentMode = Mode.SIM;

  public static final XboxController controller = new XboxController(0);
  public static final XboxController operatorController = new XboxController(1);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Intake {

    // CAN IDs
    public static final int PIVOT_ID = 32;
    public static final int SPINNER_ID = 20;

    // Sim Configs

    // Pivot
    public static final double PIVOT_GEARING = 67.5;
    public static final double MAX_PIVOT_POSITION = Units.degreesToRadians(180.0);
    public static final double PIVOT_MOI = 0.192383865;
    public static final double PIVOT_LENGTH_METERS = 0.3;

    // Spinner
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

    // CAN IDs
    public static final int LEFT_SHOOTER_ID = 15;
    public static final int RIGHT_SHOOTER_ID = 14;

    // Shooter Setpoints (RPS)
    public static final double OFF = 0.0;
    public static final double FEEDING_AMP = 25.0;
    public static final double SHOOTING = 50.0;

    // Sim Configs
    public static final double SHOOTER_GEARING = 1.5;
    public static final double SHOOTER_MOI = 0.004;
  }

  public static final class AmpBar {
    public static final double ERROR_OF_MARGIN = 0.1;

    // Motor CAN IDs
    public static final int LEFT_PIVOT_ID = 31;
    public static final int RIGHT_PIVOT_ID = 30;
    public static final int SPINNER_ID = 38;

    // Sim Configs

    // Spinner
    public static final double SPINNER_GEARING = 0.5;
    public static final double SPINNER_MOI = 0.5;

    // Pivot
    public static final double PIVOT_GEARING = 0.05;
    public static final double PIVOT_MOI = 0.05;
    public static final double PIVOT_LENGTH_METERS = 0.378;
    public static final double MAX_PIVOT_POSITION = Units.degreesToRadians(114.1633329);

    // Pivot and Spinner Setpoints

    // In RPS (Spinner Setpoints)
    public static final double SHOOTING = -0.5;
    public static final double FEEDING = -0.1;
    public static final double OFF = 0.0;

    // IN Rads (Pivot Setpoints)
    public static final double OUT = Units.degreesToRadians(50.0);
    public static final double FEEDING_POSITION = Units.degreesToRadians(45.0);
    public static final double IN = Units.degreesToRadians(0.0);
  }

  public static final class Drive {
    public static final double CONTROLLER_DEADBAND = 0.1;

    /*
     * Rotation and Translation Modifers
     * rm = rotation multiplier
     * tm = translation multipliers
     * aa = auto align
     */
    public static final double REGULAR_RM = 1.0;
    public static final double REGULAR_TM = 1.0;
    public static final double SLOW_RM = 0.5;
    public static final double SLOW_TM = 0.2;
    public static final double AA_RM = 0.8;
    public static final double AA_TM = 0.8;
    public static final double FAST_RM = 1.5;
    public static final double FAST_TM = 2.0;
  }

  public static final class Vision {
    // Stolen from Aembots vision

    public static enum CameraResolution {
      HIGH_RES,
      NORMAL
    }

    public static final double LOST_VISION_THRESHOLD = 0.5;
    // CAM = OV9281
    public static final int CAM_RES_HEIGHT = 800;
    public static final int CAM_RES_WIDTH = 1280;
    public static final Rotation2d CAM_FOV_DIAG = Rotation2d.fromDegrees(84.47);

    public static final double CAM_CALIB_PX_ERROR = 0.25;
    public static final double CAM_CALIB_STD_ERROR = 0.1;

    public static final double CAM_FPS = 40;

    public static final double CAM_LATENCY = 40;
    public static final double CAM_LATENCY_STDS = 10;

    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Transform3d frontCamToRobot =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-14.25), 0, Units.inchesToMeters(6)),
            new Rotation3d(0, Units.degreesToRadians(-67), Units.degreesToRadians(180)));

    public static final Transform3d sideCamToRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-7.19), Units.inchesToMeters(11), Units.inchesToMeters(15.25)),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(90)));

    public static final Matrix<N3, N1> highResSingleTagStdDev =
        VecBuilder.fill(0.4, 0.4, Double.MAX_VALUE);

    public static final Matrix<N3, N1> normalSingleTagStdDev =
        VecBuilder.fill(0.8, 0.8, Double.MAX_VALUE);

    public static final Matrix<N3, N1> highResMultiTagStdDev = VecBuilder.fill(0.2, 0.2, 3);

    public static final Matrix<N3, N1> normalMultiTagStdDev =
        VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
  }
}
