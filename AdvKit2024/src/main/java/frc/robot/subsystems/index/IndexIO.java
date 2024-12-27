package frc.robot.subsystems.index;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexIO {
  @AutoLog
  public static class IndexIOInputs {
    public boolean indexMotorConnected = false;
    public double indexMotorPositionRad = 0.0;
    public double indexMotorVelocityRadPerSec = 0.0;
    public double indexMotorAppliedVolts = 0.0;
    public double indexMotorCurrentAmps = 0.0;
  }

  public default void updateInputs(IndexIOInputs inputs) {}

  public default void setOutputVolts(Voltage volts) {}
}
