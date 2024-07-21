package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

public class Trigger<StateType extends SubsystemStates> {
  BooleanSupplier supplier;
  StateType state;

  public Trigger(BooleanSupplier supplier, StateType state) {
    this.supplier = supplier;
    this.state = state;
  }

  public boolean isTriggered() {
    return supplier.getAsBoolean();
  }

  public StateType getResultState() {
    return state;
  }
}
