package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Index extends SubsystemBase {
  private final IndexIO io;
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

  private final Alert indexMotorDisconnected =
      new Alert("Disconnected Index Motor.", AlertType.kError);

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

  /**
   * Instant Command: Gets the current index state.
   *
   * @return {@link IndexState}
   */
  public IndexState getIndexState() {
    return inputs.indexState;
  }

  /**
   * Instant Command: Sets the index velocity to a specific state.
   *
   * @param state - the {@link IndexState} to set the index to
   * @return {@link Command}
   */
  public Command setVelocityState(IndexState state) {
    return runOnce(() -> io.setIndexState(state));
  }

  /**
   * Continuous Command: Sets the index velocity to a specific voltage.
   *
   * <p>Default command for the index.
   *
   * @return {@link Command}
   */
  private Command manageOutput() {
    return run(
        () -> {
          switch (inputs.indexState) {
            case IDLE:
              io.setOutputVolts(Volts.of(0.0));
              break;
            case SPEAKER:
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
              setVelocityState(IndexState.IDLE);
              break;
          }
        });
  }

  /**
   * Instant Command: Sets the index state to a specific state. Used in simulation only.
   *
   * @param state - the {@link IndexState} to set the index to
   * @return {@link Command}
   */
  public void setNoteInIntake(Trigger noteInIntake) {
    if (io instanceof IndexIOSim) ((IndexIOSim) io).setNoteInIntake(noteInIntake);
  }
}
