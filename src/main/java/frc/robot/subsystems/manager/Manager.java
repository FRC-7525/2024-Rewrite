package frc.robot.subsystems.manager;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.ampBar.*;
import frc.robot.subsystems.shooter.*;;

public class Manager extends Subsystem<ManagerStates> {
    Intake intakeSubsystem;
    AmpBar ampBarSubsystem;
    Shooter shooterSubsystem;

    XboxController controller;
    XboxController operatorController;

    public Manager() {
        super("Manager", ManagerStates.IDLE);

        controller = new XboxController(0);
        operatorController = new XboxController(1);

        switch (Constants.currentMode) {
            case REAL:
                intakeSubsystem = new Intake(new IntakeIOSparkMax());
                ampBarSubsystem = new AmpBar(new AmpBarIOReal());
                shooterSubsystem = new Shooter(new ShooterIOTalonFX());
                break;
            case SIM:
                intakeSubsystem = new Intake(new IntakeIOSim());
                ampBarSubsystem = new AmpBar(new AmpBarIOSim());
                shooterSubsystem = new Shooter(new ShooterIOSim());
                break;
            default: 
                break;
        }

        addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING, () -> controller.getAButtonPressed());
        addTrigger(ManagerStates.INTAKING, ManagerStates.IDLE, () -> controller.getAButtonPressed());

        addTrigger(ManagerStates.IDLE, ManagerStates.FEED_AMP, () -> controller.getYButtonPressed());
        addTrigger(ManagerStates.FEED_AMP, ManagerStates.IDLE, () -> controller.getYButtonPressed());

        addTrigger(ManagerStates.IDLE, ManagerStates.SCORE_AMP, () -> controller.getLeftBumperPressed());
        addTrigger(ManagerStates.SCORE_AMP, ManagerStates.SCORE_AMP, () -> controller.getLeftBumperPressed());

        addTrigger(ManagerStates.IDLE, ManagerStates.SHOOTING, () -> controller.getRightBumperPressed());
        addTrigger(ManagerStates.SHOOTING, ManagerStates.IDLE, () -> controller.getRightBumperPressed());
    }

    @Override
    public void periodic() {
        super.periodic();

        intakeSubsystem.setState(getState().getIntakeState());
        ampBarSubsystem.setState(getState().getAmpBarState());
        shooterSubsystem.setState(getState().getShooterState());

        Logger.recordOutput("Manager/State", getState().getStateString());
    }

    @Override
    protected void runState() {}
}
