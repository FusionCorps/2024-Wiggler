/**
 * @file java/frc/robot/subsystems/DriveTrain.java
 * @brief Drive train is super cool
 */
package frc.robot.subsystems;

import static frc.robot.Constants.MAX_SPEED;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {

  // Motor variables
  TalonFX left_motor = new TalonFX(0);
  TalonFX right_motor = new TalonFX(1);

  // Drive variables
  private DifferentialDrive m_drive =
      new DifferentialDrive(left_motor, right_motor); // Main robot drive

  public final Trigger atFullSpeed =
      new Trigger(() -> left_motor.get() == MAX_SPEED && right_motor.get() == MAX_SPEED);

  /* Creates a new DriveTrain. */
  public DriveTrain() {
    // Configure motors - invert right side, and brake in idle mode
    right_motor
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));
    left_motor
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
  }

  /**
   * Drive the robot with an arcade drive layout.
   *
   * @param forward The forward velocity/movement
   * @param rotation The rotation to use
   */
  public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
    return runEnd(
        () ->
            m_drive.arcadeDrive(
                forward.getAsDouble() * MAX_SPEED, rotation.getAsDouble() * MAX_SPEED),
        () -> m_drive.arcadeDrive(0.0, 0.0));
  }

  public Command forwardAndBackward() {
    return arcadeDrive(() -> 1.0, () -> 0.0)
        .withTimeout(2.0)
        .andThen(Commands.waitSeconds(2.0))
        .andThen(arcadeDrive(() -> -1.0, () -> 0.0).withTimeout(2.0));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
