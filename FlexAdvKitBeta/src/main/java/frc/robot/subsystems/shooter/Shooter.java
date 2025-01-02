package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Alert topShooterDisconnected =
      new Alert("Disconnected Top Shooter Motor.", AlertType.kError);
  private final Alert bottomShooterDisconnected =
      new Alert("Disconnected Bottom Shooter Motor.", AlertType.kError);

  private final Trigger shooterAtSpeed =
      new Trigger(
          () -> {
            return MathUtil.isNear(
                    inputs.topVelocitySetpointRadPerSec,
                    inputs.topShooterVelocityRadPerSec,
                    RPM.of(500.0).in(RadiansPerSecond))
                && MathUtil.isNear(
                    inputs.bottomVelocitySetpointRadPerSec,
                    inputs.bottomShooterVelocityRadPerSec,
                    RPM.of(500.0).in(RadiansPerSecond));
          });

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    setDefaultCommand(manageVelocity());
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    topShooterDisconnected.set(!inputs.topConnected);
    bottomShooterDisconnected.set(!inputs.bottomConnected);
  }

  /**
   * Instant Command: Sets the shooter velocity to a specific state.
   *
   * @param state - the {@link ShooterState} to set the shooter to
   * @return {@link Command}
   */
  public Command setVelocityState(ShooterState state) {
    return runOnce(() -> io.setShooterState(state));
  }

  /**
   * Continuous Command: Sets the shooter velocity to a specific RPM.
   *
   * <p>Default command for the shooter.
   *
   * @return {@link Command}
   */
  private Command manageVelocity() {
    return run(
        () -> {
          switch (inputs.shooterState) {
            case IDLE:
              io.setVelocity(RPM.of(0.0), RPM.of(0.0));
              break;
            case SPEAKER:
              io.setVelocity(SPK_TOP_RPM, SPK_BOTTOM_RPM);
              break;
            case AMP:
              io.setVelocity(AMP_TOP_RPM, AMP_BOTTOM_RPM);
              break;
            case SHUTTLE:
              io.setVelocity(SHUTTLING_TOP_RPM, SHUTTLING_BOTTOM_RPM);
              break;
            case EXTAKE:
              io.setVelocity(SHOOTER_OUTTAKE_RPM, SHOOTER_OUTTAKE_RPM);
              break;
            default:
              setVelocityState(ShooterConstants.ShooterState.IDLE);
              break;
          }
        });
  }
}
