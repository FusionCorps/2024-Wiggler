package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexConstants.INDEX_AMP_PCT;
import static frc.robot.Constants.IndexConstants.INDEX_RUN_PCT;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Index extends SubsystemBase {
  public static enum IndexState {
    IDLE,
    SHOOT,
    INTAKE,
    EXTAKE,
    AMP,
  }

  private final IndexIO io;
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

  private IndexState state = IndexState.IDLE;

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

  public Command setState(IndexState state) {
    return runOnce(() -> this.state = state);
  }

  private Command manageOutput() {
    return run(
        () -> {
          switch (state) {
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
          }
        });
  }
}
