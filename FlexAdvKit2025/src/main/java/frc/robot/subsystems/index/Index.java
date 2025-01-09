package frc.robot.subsystems.index;

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
    io.updateInputs(inputs);
    Logger.processInputs("Index", inputs);

    indexMotorDisconnected.set(!inputs.indexMotorConnected);
  }

  public IndexState getIndexState() {
    return inputs.indexState;
  }

  public Command setState(IndexState state) {
    return runOnce(() -> io.setIndexState(state));
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
