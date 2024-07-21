package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.subsystems.Subsystem;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Intake extends Subsystem<IntakeStates> {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        super("Intake", IntakeStates.OFF);
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();

        //Configure PIDs here
        switch (Constants.currentMode) {
            case REAL:
                io.configurePID(new PIDController(1, 0, 0), new PIDController(1, 0, 0));
                break;
            case REPLAY:
                io.configurePID(new PIDController(1, 0, 0), new PIDController(1, 0, 0));
                break;
            case SIM:
                io.configurePID(new PIDController(1, 0, 0), new PIDController(1, 0, 0));
                break;
            default:
                break;
        }
    }

    protected void runState() {
        io.setSetpoints(getState().getPivotSetPoint(), getState().getMotorSetPoint(), getState().getUsingPID());
    }

    @Override
    public void periodic() {
        super.periodic();

        Logger.recordOutput("Intake Pose", new Pose3d(new Translation3d(0,0,0), new Rotation3d(0,io.getPosition(),0)));

        Logger.processInputs("Intake", inputs);
        io.updateInputs(inputs);
    }
}
