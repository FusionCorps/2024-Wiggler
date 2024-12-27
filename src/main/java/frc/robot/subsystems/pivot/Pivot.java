package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  public static enum PivotState {
    IDLE,
    INTAKE,
    SUBWOOFER,
    AMP,
    SHUTTLE,
    EXTAKE
  }

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  Alert pivotMainDisconnected = new Alert("Disconnected Pivot Main Motor.", AlertType.kError);
  Alert pivotFollowerDisconnected =
      new Alert("Disconnected Pivot Follower Motor.", AlertType.kError);

  @AutoLogOutput private PivotState state = PivotState.IDLE;

  public Pivot(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    setDefaultCommand(managePosition());
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    pivotMainDisconnected.set(!inputs.pivotMainConnected);
    pivotFollowerDisconnected.set(!inputs.pivotFollowerConnected);
  }

  public Command setState(PivotState state) {
    return runOnce(() -> this.state = state);
  }

  private Command managePosition() {
    return run(
        () -> {
          switch (state) {
            case IDLE:
              io.setPosition(Rotations.of(0.0));
              break;
            case INTAKE:
              io.setPosition(PIVOT_STOW_POS);
              break;
            case SUBWOOFER:
              io.setPosition(PIVOT_SUB_POS);
              break;
            case AMP:
              io.setPosition(PIVOT_AMP_POS);
              break;
            case SHUTTLE:
              io.setPosition(PIVOT_SHUTTLING_POS);
              break;
            case EXTAKE:
              io.setPosition(PIVOT_STOW_POS);
              break;
            default:
              io.setPosition(PIVOT_STOW_POS);
              break;
          }
        });
  }
}
