package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.manager.ManagerStates;

public class ShootNearSpeakerCommand extends Command {
    // Kinda an auto command, but it has a is finished condition soooo too bad so sad

    Robot robot = null;
    public boolean shot = false;

    public ShootNearSpeakerCommand(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.managerSubsystem.setState(ManagerStates.SPINNING_UP);
        shot = false;
    }

    @Override
    public void execute() {
        if (robot.managerSubsystem.driveNearSetPose(
                (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.AutoAlign.blueSpeakerPose
                        : Constants.AutoAlign.redSpeakerPose))
                && !shot
                && robot.managerSubsystem.intakeNearSetpoints()) {
            // Good practice to do this? Prob not. Not too important so idc
            robot.managerSubsystem.setState(ManagerStates.SHOOTING);
            shot = true;
        }
    }

    @Override
    public boolean isFinished() {
        return shot && robot.managerSubsystem.getState() == ManagerStates.IDLE;
    }
}
