package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double intakePositionRad = 0.0;
    public double intakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setOutputPercent(double output) {}
}
