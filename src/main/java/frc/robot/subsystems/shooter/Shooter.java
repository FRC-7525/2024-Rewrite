package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;
  private SysIdRoutine sysId;
  private ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(1.0, 0, 0);
        io.configureFF(1, 0, 0);
        break;
      case REPLAY:
        io.configurePID(1.0, 0.0, 0.0);
        io.configureFF(1, 0, 0);
        break;
      case SIM:
        io.configurePID(0.5, 0.0, 0.0);
        io.configureFF(1, 0, 0);
        break;
      default:
        break;
    }

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void setSpeed(Double speed) {
    io.setSpeed(speed);
  }

  public Command setSpeedCommand(DoubleSupplier rps) {
    return run(() -> setSpeed(rps.getAsDouble()));
  }

  public Command getDefaultCommand() {
    return setSpeedCommand(() -> 0.0);
  }

  public boolean nearSpeedPoint() {
    return io.nearSpeedPoint();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput(
        "Shooter/Running Command",
        Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));
  }
}
