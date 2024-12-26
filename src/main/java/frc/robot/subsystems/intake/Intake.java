package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.INTAKE_OUTPUT_PERCENT;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert intakeDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;

    intakeDisconnectedAlert = new Alert("Disconnected intake motor.", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeDisconnectedAlert.set(!inputs.connected);
  }

  public Command runIntake(boolean reverse) {
    return runEnd(
        () -> io.setOutputVolts(Volts.of(INTAKE_OUTPUT_PERCENT * 12.0 * (reverse ? -1 : 1))),
        () -> io.setOutputVolts(Volts.of(0.0)));
  }
}
