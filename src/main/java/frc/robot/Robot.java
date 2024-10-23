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

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.AutoCommands;
// import frc.robot.commands.ShootNearSpeakerCommand;
import frc.robot.commands.Shooting;
import frc.robot.subsystems.manager.*;
import frc.robot.util.NoteSimulator;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

	public Manager managerSubsystem;
	private SendableChooser<String> autoChooser;
	private AutoCommands autoCommands = new AutoCommands(this);
	private Command autonomousCommand;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Record metadata
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0:
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		// Set up data receivers & replay source
		switch (Constants.currentMode) {
			case REAL:
				// Running on a real robot, log to a USB stick ("/U/logs")
				// Logger.addDataReceiver(new WPILOGWriter());
				CameraServer.startAutomaticCapture();
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case SIM:
				// Running a physics simulator, log to NT
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case REPLAY:
				// Replaying a log, set up replay source
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(
					new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
				);
				break;
		}

		// See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
		// Logger.disableDeterministicTimestamps()

		// Start AdvantageKit logger
		Logger.start();

		managerSubsystem = new Manager();

		NamedCommands.registerCommand("Intaking", autoCommands.intaking());
		NamedCommands.registerCommand("Shooting", new Shooting(this));
		NamedCommands.registerCommand("Return To Idle", autoCommands.returnToIdle());
		NamedCommands.registerCommand("Speeding Up", autoCommands.startSpinningUp());
		NamedCommands.registerCommand("Spin and Intake", autoCommands.spinAndIntake());
		NamedCommands.registerCommand("Shoot Near Speaker", new PrintCommand("lalal"));

		/* Not using this bc PP doesen't let you put "|, :, etc." in the Auto name so we wouldnt
		 * be able to use the same names as the ones established in our auto style guide thing.
		 * The chooser also puts all the commands to NT4 (I think) which isn't that big of a deal
		 * but strings are better trust
		 * autoChooser = AutoBuilder.buildAutoChooser();
		 */

		autoChooser = new SendableChooser<String>();

		// Misc Autos
		autoChooser.addOption("0: Start Anywhere (NO VISION) | Cross Line", "Drive Forwards");
		autoChooser.addOption("0: Start Anywhere | Do Nothing", "Do Nothing");

		// 1 Note Autos
		autoChooser.addOption("1: Start Middle | Preload", "Drive Backwards + Score");

		// 2 Note Autos
		autoChooser.addOption("2: Start Amp | CA", "Left Note");
		autoChooser.addOption("2: Start Middle | CM", "Middle Note");
		autoChooser.addOption("2: Start Source | CS", "Right Note");

		// 3 Note Autos
		autoChooser.addOption("3: Start Source | FR, FMS", "2 Far Right");
		autoChooser.addOption("3: Start Amp | CL, CM", "CloseTwoLeft");
		autoChooser.addOption("3: Start Amp | CL, FL", "All Left");

		// 4 Note Autos
		autoChooser.addOption("4: Start Middle | All Close", "All Close");
		autoChooser.addOption("4: Start Middle | CA, CM, FA", "Optimized 4 Note Auto");
		autoChooser.addOption("4: Start Source | CS, FS, FMS", "CSFSFSM");
		autoChooser.addOption("4: Start Source | FS, FMS, FM", "3 Center Line");

		// 5 Note Auto
		autoChooser.addOption("5: Start Middle | CA, CM, FL, FMA", "Left 5 Note");
		autoChooser.addOption("5: Start Middle | All Close, FM", "All Close + FM");
		autoChooser.addOption("5: Start Middle | CS, MC, FM, FMA", "Center 5 Note");
		autoChooser.addOption("5: Start Middle | All Close, FA", "Optimized 5 Note Auto");

		SmartDashboard.putData("Auto autoChooser", autoChooser);
	}

	/** This function is called periodically during all modes. */
	@Override
	public void robotPeriodic() {
		managerSubsystem.periodic();

		// Logs note sim logging and updating sims
		// NoteSimulator.update();
		// NoteSimulator.logNoteInfo();

		CommandScheduler.getInstance().run();
	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		managerSubsystem.stop();
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {}

	/** This function is called once the robot enters Auto. */
	@Override
	public void autonomousInit() {
		autonomousCommand = new PathPlannerAuto(autoChooser.getSelected());

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {}
}
