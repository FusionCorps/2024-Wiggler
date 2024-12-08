package frc.robot.subsystems.intake;

import static frc.robot.Constants.driverController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;

    driverController.a().onTrue(intakeCommand());
  }

  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command intakeCommand() {
    return Commands.sequence(
        runOnce(() -> intakeIO.setIntakeVoltage(2.0)),
        Commands.waitSeconds(2.0),
        runOnce(() -> intakeIO.setIntakeVoltage(0.0)));
  }
}
