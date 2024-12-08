package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double leftDriveMotorPosRad = 0.0;
    public double leftDriveMotorAppliedVolts = 0.0;
    public double leftDriveMotorVelocityRadPerSec = 0.0;

    public double rightDriveMotorPosRad = 0.0;
    public double rightDriveMotorVelocityRadPerSec = 0.0;
    public double rightDriveMotorAppliedVolts = 0.0;
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setDriveVelocity(
      double leftVelocityRadPerSec, double rightVelocityRadPerSec) {}

  public default void setDriveVoltage(double leftVolts, double rightVolts) {}
}
