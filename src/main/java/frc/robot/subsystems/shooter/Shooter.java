package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.util.NoteSimulator;

import org.littletonrobotics.junction.Logger;

public class Shooter extends Subsystem<ShooterStates> {

    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;

    public Shooter(ShooterIO io) {
        super("Shooter", ShooterStates.OFF);
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
                io.configurePID(1.0, 0, 0);
                break;
            case REPLAY:
                io.configurePID(1.0, 0.0, 0.0);
                break;
            case SIM:
                io.configurePID(0.5, 0.0, 0.0);
                break;
            default:
                break;
        }
    }

    @Override
    protected void runState() {
        io.setSpeed(getState().getMotorSpeedpoint());
        if (getState() == ShooterStates.SHOOTING)
            NoteSimulator.launch(0); //TODO: add velocity
    }

    public boolean nearSpeedPoint() {
        return io.nearSpeedPoint();
    }

    @Override
    public void periodic() {
        super.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }
}
