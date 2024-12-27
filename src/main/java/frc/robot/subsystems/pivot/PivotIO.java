package frc.robot.subsystems.pivot;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean pivotMainConnected = false;
    public double pivotMainPositionRad = 0.0;
    public double pivotMainVelocityRadPerSec = 0.0;
    public double pivotMainAppliedVolts = 0.0;
    public double pivotMainCurrentAmps = 0.0;

    public boolean pivotFollowerConnected = false;
    public double pivotFollowerPositionRad = 0.0;
    public double pivotFollowerVelocityRadPerSec = 0.0;
    public double pivotFollowerAppliedVolts = 0.0;
    public double pivotFollowerCurrentAmps = 0.0;

    public double pivotPositionSetpointRad = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setPosition(Angle position) {}
}
