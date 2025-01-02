package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.INTAKE_OUTPUT_PERCENT;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
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
    setDefaultCommand(manageIntake());
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeDisconnectedAlert.set(!inputs.intakeMotorConnected);
    beamBreakDisconnected.set(!inputs.beamBreakSensorConnected);
  }

  /**
   * Instant Command: Sets the intake velocity to a specific state.
   *
   * @param state - the {@link IntakeState} to set the intake to
   * @return {@link Command}
   */
  public Command setVelocityState(IntakeState state) {
    return runOnce(() -> io.setIntakeState(state));
  }

  /**
   * Continuous Command: Sets the intake velocity to a specific percentage.
   *
   * <p>Default command for the intake.
   *
   * @param pct - the percentage to set the intake to
   * @return {@link Command}
   */
  private Command manageIntake() {
    return run(
        () -> {
          switch (inputs.intakeState) {
            case IDLE:
              io.setOutputVolts(Volts.of(0.0));
              break;
            case INTAKE:
              io.setOutputVolts(Volts.of(INTAKE_OUTPUT_PERCENT * 12.0));
              break;
            case EXTAKE:
              io.setOutputVolts(Volts.of(-INTAKE_OUTPUT_PERCENT * 12.0));
              break;
            default:
              setVelocityState(IntakeConstants.IntakeState.IDLE);
              break;
          }
        });
  }
}
