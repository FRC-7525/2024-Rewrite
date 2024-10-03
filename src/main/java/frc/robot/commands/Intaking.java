package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.manager.ManagerStates;

public class Intaking extends Command {
    Robot robot = null;

    public Intaking(Robot robot) {
        this.robot = robot;
    }
    @Override
    public void initialize() {
        robot.managerSubsystem.setState(ManagerStates.SPINNING_UP);
    }

    @Override
    public boolean isFinished() {
        return robot.managerSubsystem.getState() == ManagerStates.IDLE;
    }
}
