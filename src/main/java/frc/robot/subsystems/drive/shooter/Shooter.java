package frc.robot.subsystems.drive.shooter;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static enum ShooterState {
    IDLE,
    SPEAKER,
    AMP,
    SHUTTLE,
    EXTAKE
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  @AutoLogOutput private ShooterState state = ShooterState.IDLE;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setState(ShooterState state) {
    this.state = state;
  }

  public Command shoot() {
    return run(
        () -> {
          switch (state) {
            case IDLE:
              io.setVelocity(RPM.of(0.0), RPM.of(0.0));
              break;
            case SPEAKER:
              io.setVelocity(SPK_TOP_RPM, SPK_BOTTOM_RPM);
              break;
            case AMP:
              io.setVelocity(AMP_TOP_RPM, AMP_BOTTOM_RPM);
              break;
            case SHUTTLE:
              io.setVelocity(SHUTTLING_TOP_RPM, SHUTTLING_BOTTOM_RPM);
              break;
            case EXTAKE:
              io.setVelocity(SHOOTER_OUTTAKE_RPM, SHOOTER_OUTTAKE_RPM);
              break;
            default:
              io.setVelocity(RPM.of(0.0), RPM.of(0.0));
              break;
          }
        });
  }
}
