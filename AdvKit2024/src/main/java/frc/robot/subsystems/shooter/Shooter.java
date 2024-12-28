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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static enum ShooterState {
    IDLE,
    SPEAKER,
    AMP,
    SHUTTLE,
    EXTAKE
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  Alert topShooterDisconnected = new Alert("Disconnected Top Shooter Motor.", AlertType.kError);
  Alert bottomShooterDisconnected =
      new Alert("Disconnected Bottom Shooter Motor.", AlertType.kError);

  @AutoLogOutput private ShooterState state = ShooterState.IDLE;

  Trigger shooterAtSpeed =
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

  public Command setState(ShooterState state) {
    return runOnce(() -> this.state = state);
  }

  public ShooterState getState() {
    return state;
  }

  private Command manageVelocity() {
    return run(
        () -> {
          switch (state) {
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
              io.setVelocity(RPM.of(0.0), RPM.of(0.0));
              break;
          }
        });
  }
}
