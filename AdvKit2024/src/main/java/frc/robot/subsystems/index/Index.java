package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Index extends SubsystemBase {
  private final IndexIO io;
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

  Alert indexMotorDisconnected = new Alert("Disconnected Index Motor.", AlertType.kError);

  public Index(IndexIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    setDefaultCommand(manageOutput());
    io.updateInputs(inputs);
    Logger.processInputs("Index", inputs);

    indexMotorDisconnected.set(!inputs.indexMotorConnected);
  }

  public Command setVelocityState(IndexIO.IndexState state) {
    return runOnce(() -> io.setIndexState(state));
  }

  private Command manageOutput() {
    return run(
        () -> {
          switch (inputs.indexState) {
            case IDLE:
              io.setOutputVolts(Volts.of(0.0));
              break;
            case SHOOT:
              io.setOutputVolts(Volts.of(INDEX_RUN_PCT * 12.0));
              break;
            case INTAKE:
              io.setOutputVolts(Volts.of(INDEX_RUN_PCT * 12.0));
              break;
            case EXTAKE:
              io.setOutputVolts(Volts.of(-INDEX_RUN_PCT * 12.0));
              break;
            case AMP:
              io.setOutputVolts(Volts.of(INDEX_AMP_PCT * 12.0));
              break;
            default:
              setVelocityState(IndexIO.IndexState.IDLE);
              break;
          }
        });
  }
}
