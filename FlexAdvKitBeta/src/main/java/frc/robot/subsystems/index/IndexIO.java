package frc.robot.subsystems.index;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IndexConstants.IndexState;
import org.littletonrobotics.junction.AutoLog;

public interface IndexIO {
  @AutoLog
  public static class IndexIOInputs {
    public boolean indexMotorConnected = false;
    public double indexPositionRad = 0.0;
    public double indexVelocityRadPerSec = 0.0;
    public double indexAppliedVolts = 0.0;
    public double indexCurrentAmps = 0.0;
    public IndexState indexState = IndexState.IDLE;
  }

  public default void updateInputs(IndexIOInputs inputs) {}

  public default void setOutputVolts(Voltage volts) {}

  public default void setIndexState(IndexState state) {}
}
