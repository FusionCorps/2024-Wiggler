package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeMotorConnected = false;
    public double intakePositionRad = 0.0;
    public double intakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;

    public boolean beamBreakSensorConnected = false;
    public boolean noteInIntake = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setOutputVolts(Voltage volts) {}
}
