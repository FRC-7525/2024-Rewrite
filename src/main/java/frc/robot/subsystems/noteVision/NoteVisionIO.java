package frc.robot.subsystems.noteVision;

import org.littletonrobotics.junction.AutoLog;


public interface NoteVisionIO {
  @AutoLog
  public static class NoteVisionIOInputs {
    public boolean hasTarget = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(NoteVisionIOInputs inputs) {}
}


