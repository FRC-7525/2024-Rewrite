package frc.robot.subsystems.manager;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ampBar.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;

public class Manager extends Subsystem<ManagerStates> {
  Intake intakeSubsystem;
  AmpBar ampBarSubsystem;
  Shooter shooterSubsystem;
  Drive drive;

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
        drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        break;
      case REPLAY:
        intakeSubsystem = new Intake(new IntakeIO() {});
        ampBarSubsystem = new AmpBar(new AmpBarIO() {});
        shooterSubsystem = new Shooter(new ShooterIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
      case SIM:
        intakeSubsystem = new Intake(new IntakeIOSim());
        ampBarSubsystem = new AmpBar(new AmpBarIOSim());
        shooterSubsystem = new Shooter(new ShooterIOSim());
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        break;
      default:
        break;
    }

    // State Transitions (Nothing Automatic YET)

    /* Generally each action has a specific button, in intermediary states X will return to idle and you press
       the specific button to go through the states. Right now you can go through states without them being finished
       which should not be the case. Some state transitions that can be automated are also not automated. Most
    of the functions required for the TODOs are already build into the IOs!*/

    // TODO: Automatically return to idle after scoring amp (using a timer)
    // TODO: Automatically current sense notes to return to idle after intaking
    // TODO: Automatically go to the shooting state after spinning up if the shooting state is
    // entered from main controller input

    // Intaking (B)
    addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING, () -> controller.getBButtonPressed());
    addTrigger(ManagerStates.INTAKING, ManagerStates.IDLE, () -> controller.getBButtonPressed());

    // Amping (Y)
    addTrigger(ManagerStates.IDLE, ManagerStates.FEED_AMP, () -> controller.getYButtonPressed());
    addTrigger(
        ManagerStates.FEED_AMP, ManagerStates.SCORE_AMP, () -> controller.getYButtonPressed());
    addTrigger(ManagerStates.SCORE_AMP, ManagerStates.IDLE, () -> controller.getYButtonPressed());
    addTrigger(ManagerStates.FEED_AMP, ManagerStates.IDLE, () -> controller.getXButtonPressed());

    // Shooting (A)
    addTrigger(ManagerStates.IDLE, ManagerStates.SPINNING_UP, () -> controller.getAButtonPressed());
    addTrigger(
        ManagerStates.IDLE,
        ManagerStates.SPINNING_UP,
        () -> operatorController.getAButtonPressed());
    addTrigger(ManagerStates.SPINNING_UP, ManagerStates.IDLE, () -> controller.getXButtonPressed());
    addTrigger(
        ManagerStates.SPINNING_UP, ManagerStates.SHOOTING, () -> controller.getAButtonPressed());
    addTrigger(ManagerStates.SHOOTING, ManagerStates.IDLE, () -> controller.getAButtonPressed());

    // Cancel Actions
    addTrigger(
        ManagerStates.INTAKING, ManagerStates.IDLE, () -> operatorController.getXButtonPressed());
    addTrigger(
        ManagerStates.FEED_AMP, ManagerStates.IDLE, () -> operatorController.getXButtonPressed());
    addTrigger(
        ManagerStates.SCORE_AMP, ManagerStates.IDLE, () -> operatorController.getXButtonPressed());
    addTrigger(
        ManagerStates.SPINNING_UP,
        ManagerStates.IDLE,
        () -> operatorController.getXButtonPressed());
    addTrigger(
        ManagerStates.SHOOTING, ManagerStates.IDLE, () -> operatorController.getXButtonPressed());

    // Drive Configs
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));
  }

  @Override
  public void periodic() {
    super.periodic();

    intakeSubsystem.setState(getState().getIntakeState());
    ampBarSubsystem.setState(getState().getAmpBarState());
    shooterSubsystem.setState(getState().getShooterState());

    intakeSubsystem.periodic();
    ampBarSubsystem.periodic();
    shooterSubsystem.periodic();
  }

  public void stop() {
    intakeSubsystem.stop();
    ampBarSubsystem.stop();
    shooterSubsystem.stop();
  }

  @Override
  protected void runState() {}
}
