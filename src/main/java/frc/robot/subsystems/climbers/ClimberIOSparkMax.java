package frc.robot.subsystems.climbers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;

public class ClimberIOSparkMax implements ClimberIO {
    
    private CANSparkMax leftClimberMotor;
    private CANSparkMax rightClimberMotor;

    private RelativeEncoder leftClimberEncoder;
    private RelativeEncoder rightClimberEncoder;

    private PIDController climberController;

    private LinearFilter leftFilter;
    private LinearFilter rightFilter;

    private boolean leftClimberZeroed;
    private boolean rightClimberZeroed;

    private double leftClimberSetpoint;
    private double rightClimberSetpoint;

    private double leftAppliedVolts;
    private double rightAppliedVolts;

    public ClimberIOSparkMax() {
        leftClimberMotor = new CANSparkMax(Constants.Climber.LEFT_ID, CANSparkMax.MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.Climber.RIGHT_ID, CANSparkMax.MotorType.kBrushless);

        leftClimberMotor.setInverted(false);
        rightClimberMotor.setInverted(true);

        leftClimberEncoder = leftClimberMotor.getEncoder();
        rightClimberEncoder = rightClimberMotor.getEncoder();

        climberController.setPID(0, 0, 0);

        leftFilter = LinearFilter.movingAverage(5);
        rightFilter = LinearFilter.movingAverage(5);

        leftClimberSetpoint = 0;
        rightClimberSetpoint = 0;

        leftClimberZeroed = false;
        rightClimberZeroed = false;

        leftAppliedVolts = 0;
        rightAppliedVolts = 0;
    }

    public void updateInput(ClimberIOInputs inputs) {
        inputs.leftClimberPosition = leftClimberEncoder.getPosition();
        inputs.rightClimberPosition = rightClimberEncoder.getPosition();
        inputs.leftClimberSetpoint = leftClimberSetpoint;
        inputs.rightClimberSetpoint = rightClimberSetpoint;
        inputs.leftClimberSpeed = leftClimberEncoder.getVelocity()/60;
        inputs.rightClimberSpeed = rightClimberEncoder.getVelocity()/60;
    }

    public void updateOutputs(ClimberIOOutputs outputs) {
        outputs.leftClimberAppliedVoltage = leftAppliedVolts;
        outputs.rightClimberAppliedVoltage = rightAppliedVolts;
    }

    public boolean nearSetpoints() {
        return 
            Math.abs(leftClimberEncoder.getPosition() - leftClimberEncoder.getPosition()) < Constants.Climber.ERROR_OF_MARGIN && 
            Math.abs(rightClimberEncoder.getPosition() - rightClimberEncoder.getPosition()) < Constants.Climber.ERROR_OF_MARGIN;
    }

    public boolean climbersZeroed() {
        return leftClimberZeroed && rightClimberZeroed;
    }

    public void configurePID(double kP, double kI, double kD) {
        climberController.setPID(kP, kI, kD);
    }

    public void setSetpoints(double leftSetpoint, double rightSetpoint) {
        this.leftClimberSetpoint = leftSetpoint;
        this.rightClimberSetpoint = rightSetpoint;

        this.leftAppliedVolts = climberController.calculate(leftClimberEncoder.getPosition(), leftSetpoint);
        this.rightAppliedVolts = climberController.calculate(leftClimberEncoder.getPosition(), rightSetpoint);

        leftClimberMotor.set(leftAppliedVolts);
        rightClimberMotor.set(rightAppliedVolts);
    }

    public void zeroClimbers() {
        double leftZeroingSpeed = -0.25;
        double rightZeroingSpeed = -0.25;

        // Not voltage sensing 😮‍💨
        if (leftFilter.calculate(leftClimberMotor.getOutputCurrent()) > Constants.Climber.LEFT_CURRENT_LIMIT || leftClimberZeroed) {
            leftZeroingSpeed = 0;
            if (!leftClimberZeroed) leftClimberMotor.set(0);
            leftClimberZeroed = true;
        }

        if (rightFilter.calculate(rightClimberMotor.getOutputCurrent()) > Constants.Climber.RIGHT_CURRENT_LIMIT || rightClimberZeroed) {
            rightZeroingSpeed = 0;
            if (!rightClimberZeroed) rightClimberMotor.set(0);
            rightClimberZeroed = true;
        }

        leftClimberMotor.set(leftZeroingSpeed);
        rightClimberMotor.set(rightZeroingSpeed);
    }

    public void stop() {
        leftClimberMotor.stopMotor();
        rightClimberMotor.stopMotor();
    }
}
