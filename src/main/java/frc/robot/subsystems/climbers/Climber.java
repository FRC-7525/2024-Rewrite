package frc.robot.subsystems.climbers;

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
