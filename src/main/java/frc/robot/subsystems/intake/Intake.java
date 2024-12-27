package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.INTAKE_OUTPUT_PERCENT;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static enum IntakeState {
    IDLE,
    INTAKE,
    EXTAKE,
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Trigger noteInIntake =
      new Trigger(() -> inputs.beamBreakSensorConnected && inputs.noteInIntake);

  private IntakeState state = IntakeState.IDLE;

  private final Alert intakeDisconnectedAlert =
      new Alert("Disconnected intake motor.", AlertType.kError);
  private final Alert beamBreakDisconnected =
      new Alert("Disconnected Beam Break Sensor.", AlertType.kError);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    setDefaultCommand(manageIntake());
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeDisconnectedAlert.set(!inputs.intakeMotorConnected);
    beamBreakDisconnected.set(!inputs.beamBreakSensorConnected);
  }

  public Command setState(IntakeState state) {
    return runOnce(() -> this.state = state);
  }

  private Command manageIntake() {
    return run(
        () -> {
          switch (state) {
            case IDLE:
              io.setOutputVolts(Volts.of(0.0));
              break;
            case INTAKE:
              io.setOutputVolts(Volts.of(INTAKE_OUTPUT_PERCENT * 12.0));
              break;
            case EXTAKE:
              io.setOutputVolts(Volts.of(-INTAKE_OUTPUT_PERCENT * 12.0));
              break;
          }
        });
  }
}
