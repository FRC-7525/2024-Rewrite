package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.manager.ManagerStates;

public class AutoCommands {

	Robot robot = null;

	public AutoCommands(Robot robot) {
		this.robot = robot;
	}

	public Command intaking() {
		return new InstantCommand(() -> robot.managerSubsystem.setState(ManagerStates.INTAKING));
	}

	// public Command shooting() {
	// 	return new InstantCommand(() -> robot.managerSubsystem.setState(ManagerStates.SHOOTING));
	// }

	public Command returnToIdle() {
		return new InstantCommand(() -> robot.managerSubsystem.setState(ManagerStates.IDLE));
	}

	public Command startSpinningUp() {
		return new InstantCommand(() ->
			robot.managerSubsystem.setState(ManagerStates.OPERATOR_SPINNING_UP)
		);
	}

	public Command spinAndIntake() {
		return new InstantCommand(() ->
			robot.managerSubsystem.setState(ManagerStates.SPINNING_AND_INTAKING)
		);
	}
}
