package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public enum IntakeStates implements SubsystemStates {
    OFF(
        "Off",
        Constants.Intake.OFF,
        Constants.Intake.OFF,
        true
    ),
    INTAKING(
        "Intaking",
        Constants.Intake.DOWN,
        Constants.Intake.ON,
        false
    ),
    FEEDING(
        "Feeding",
        Constants.Intake.OFF, 
        Constants.Intake.REVERSE,
        true
    ),
    OUTTAKING(
        "Outtaking",
        Constants.Intake.DOWN, 
        Constants.Intake.REVERSE,
        false
    );

    private String stateString;
    private double pivotMotorSetpoint;
    private double intakeMotorSetpoint;
    private boolean useIn;

    IntakeStates(String stateString, double pivotMotorSetpoint, double intakeMotorSetpoint, boolean useIn) {
        this.stateString = stateString;
        this.pivotMotorSetpoint = pivotMotorSetpoint;
        this.intakeMotorSetpoint = intakeMotorSetpoint;
        this.useIn = useIn;
    }

    public String getStateString() {
        return this.stateString;
    }

    public double getPivotSetPoint() {
        return pivotMotorSetpoint;
    }

    public double getMotorSetPoint() {
        return intakeMotorSetpoint;
    }

    public boolean getUsingPID() {
        return useIn;
    }
}