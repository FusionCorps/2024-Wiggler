package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakePosRad = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeVelocity(double velocityRadPerSec) {}

  public default void setIntakeVoltage(double volts) {}
}
