package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants.IntakeState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public final Trigger noteInIntake =
      new Trigger(() -> inputs.beamBreakSensorConnected && inputs.noteInIntake);

  private final Alert intakeDisconnectedAlert =
      new Alert("Disconnected intake motor.", AlertType.kError);
  private final Alert beamBreakDisconnected =
      new Alert("Disconnected Beam Break Sensor.", AlertType.kError);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeDisconnectedAlert.set(!inputs.intakeMotorConnected);
    beamBreakDisconnected.set(!inputs.beamBreakSensorConnected);
  }

  public Command setState(IntakeState state) {
    return runOnce(() -> io.setIntakeState(state));
  }
}
