package frc.robot.subsystems.manager;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.exampleSubsystem.Example;
import frc.robot.subsystems.exampleSubsystem.ExampleIO;
import frc.robot.subsystems.exampleSubsystem.ExampleIOSim;
import frc.robot.subsystems.exampleSubsystem.ExampleIOSparkMax;
import frc.robot.subsystems.exampleSubsystem.ExampleStates;

public class Manager extends Subsystem<ManagerStates> {
    
    Example exampleSubsystem;
    // There is a xbox controlelr sim ya'll can use if interested
    XboxController controller;
    XboxController operatorController;

    public Manager() {
        super("Manager", ManagerStates.IDLE);

        controller = new XboxController(0);
        operatorController = new XboxController(1);

        switch (Constants.currentMode) {
            case REAL:
                exampleSubsystem = new Example(new ExampleIOSim());
            case SIM:
                exampleSubsystem = new Example(new ExampleIOSparkMax());
            default:
                exampleSubsystem = new Example(new ExampleIO() {});
        }

        // Idle -> Exampling transitions
        addTrigger(ManagerStates.IDLE, ManagerStates.EXAMPLING, () -> controller.getAButtonPressed());
        addTrigger(ManagerStates.EXAMPLING, ManagerStates.IDLE, () -> controller.getAButtonPressed());

        // Idle -> half speed transitions
        addTrigger(ManagerStates.IDLE, ManagerStates.HALF_SPEEDING, () -> controller.getYButtonPressed());
        addTrigger(ManagerStates.HALF_SPEEDING, ManagerStates.IDLE, () -> controller.getYButtonPressed());
    }

    @Override
    public void periodic() {
        super.periodic();

        if (getState() == ManagerStates.IDLE) {
            exampleSubsystem.setState(ExampleStates.OFF);
        } else if (getState() == ManagerStates.HALF_SPEEDING) {
            exampleSubsystem.setState(ExampleStates.HALF_SPEED);
        } else if (getState() == ManagerStates.EXAMPLING) {
            exampleSubsystem.setState(ExampleStates.ON);
        }

        Logger.recordOutput("Manager/State", getState().getStateString());
    }

	@Override
	protected void runState() {
    }

}
