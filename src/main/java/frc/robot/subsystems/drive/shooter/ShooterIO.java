package frc.robot.subsystems.drive.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean topConnected = false;
    public double topShooterPositionRad = 0.0;
    public double topShooterVelocityRadPerSec = 0.0;
    public double topShooterAppliedVolts = 0.0;
    public double topShooterCurrentAmps = 0.0;

    public boolean bottomConnected = false;
    public double bottomShooterPositionRad = 0.0;
    public double bottomShooterVelocityRadPerSec = 0.0;
    public double bottomShooterAppliedVolts = 0.0;
    public double bottomShooterCurrentAmps = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVelocity(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {}
}
