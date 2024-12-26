package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.INTAKE_OUTPUT_PERCENT;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    intakeDisconnectedAlert.set(!inputs.connected);
  }

  public Command runIntake(double outputPercent) {
    return runEnd(() -> io.setOutputPercent(INTAKE_OUTPUT_PERCENT), () -> io.setOutputPercent(0.0));
  }
}
