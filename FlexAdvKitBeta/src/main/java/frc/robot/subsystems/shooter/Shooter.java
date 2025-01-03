package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants.ShooterState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Alert topShooterDisconnected =
      new Alert("Disconnected Top Shooter Motor.", AlertType.kError);
  private final Alert bottomShooterDisconnected =
      new Alert("Disconnected Bottom Shooter Motor.", AlertType.kError);

  @AutoLogOutput
  public final Trigger atSpeed =
      new Trigger(
          () ->
              MathUtil.isNear(
                      inputs.topVelocitySetpointRadPerSec,
                      inputs.topShooterVelocityRadPerSec,
                      RPM.of(500.0).in(RadiansPerSecond))
                  && MathUtil.isNear(
                      inputs.bottomVelocitySetpointRadPerSec,
                      inputs.bottomShooterVelocityRadPerSec,
                      RPM.of(500.0).in(RadiansPerSecond)));

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    topShooterDisconnected.set(!inputs.topConnected);
    bottomShooterDisconnected.set(!inputs.bottomConnected);
  }

  public Command setState(ShooterState state) {
    return runOnce(() -> io.setShooterState(state));
  }
}
