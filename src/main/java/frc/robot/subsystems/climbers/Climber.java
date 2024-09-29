package frc.robot.subsystems.climbers;

import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.climbers.ClimberIO.ClimberIOOutputs;

public class Climber extends Subsystem<ClimberStates> {

    ClimberIO io;
    ClimberIOInputsAutoLogged inputs;
    ClimberIOOutputs outputs;
    
    boolean zeroed;

    public Climber(ClimberIO io) {
        super("Climber", ClimberStates.OFF);
        this.io = io;
        inputs = new ClimberIOInputsAutoLogged();
        outputs = new ClimberIOOutputs();
        zeroed = false;

        switch (Constants.currentMode) {
            case REAL:
                io.configurePID(Constants.Climber.REAL_PID.kP, Constants.Climber.REAL_PID.kI, Constants.Climber.REAL_PID.kD);
                break;
            case SIM:
                io.configurePID(Constants.Climber.SIM_PID.kP, Constants.Climber.SIM_PID.kI, Constants.Climber.SIM_PID.kD);
                break;
            case REPLAY:
                io.configurePID(0, 0, 0);
                break;
            default:
                break;
        }
    }

    @Override
    protected void runState() {
        io.updateInputs(inputs);
        io.updateOutputs(outputs);

        if (zeroed) {
            io.setSetpoints(getState().getLeftSetpoint(), getState().getRightSetpoint());
        } else {
            io.zeroClimbers();
            zeroed = io.climbersZeroed();
        }
    }

    public void stop() {
        io.stop();
    }

    public boolean nearSetpoints() {
        return io.nearSetpoints();
    }    
}
