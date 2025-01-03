package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PivotState;
import java.util.function.Supplier;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  @AutoLogOutput
  public Trigger atAngle =
      new Trigger(
          () -> MathUtil.isNear(inputs.pivotMainPositionRad, getPosition().in(Radians), 0.001));

  @AutoLogOutput private final LoggedMechanism2d pivotMech = new LoggedMechanism2d(2, 2);
  private final LoggedMechanismLigament2d pivotArm =
      pivotMech
          .getRoot("pivotMech", 1, 1)
          .append(new LoggedMechanismLigament2d("pivotarm", 0.5, 0));

  private final Alert pivotMainDisconnected =
      new Alert("Disconnected Pivot Main Motor.", AlertType.kError);
  private final Alert pivotFollowerDisconnected =
      new Alert("Disconnected Pivot Follower Motor.", AlertType.kError);

  public Pivot(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    pivotArm.setAngle(Rotation2d.fromRadians(inputs.pivotMainPositionRad));

    pivotMainDisconnected.set(!inputs.pivotMainConnected);
    pivotFollowerDisconnected.set(!inputs.pivotFollowerConnected);
  }

  public Command setState(PivotState state) {
    return runOnce(() -> io.setState(state));
  }

  /**
   * @return the current pivot angle as a {@link Angle} object
   */
  public Angle getPosition() {
    return Radians.of(inputs.pivotMainPositionRad);
  }

  /**
   * @return the current pivot state as a {@link PivotState} object
   */
  public PivotState getState() {
    return inputs.pivotState;
  }

  /**
   * Continuous command: runs the pivot at specific duty cycle and stops it when the command ends
   *
   * @param pct - duty cycle percentage
   * @return "run-end" {@link RunCommand}
   */
  public Command setPivotPctCommand(double pct) {
    return setState(PivotState.MANUAL_OVERRIDE)
        .andThen(runEnd(() -> io.setPct(pct), () -> io.setPct(0.0)));
  }

  /**
   * Continuous command: moves the pivot to a specific angle and stops it when the command ends The
   * angle is based on the robot's distance from the speaker and a lookup table ACCURATE
   * localization is necessary for this to work well
   *
   * @param drivePoseSupplier - a supplier for the robot's current pose
   * @return {@link SequentialCommandGroup}
   */
  public Command angleToSpeakerCommand(Supplier<Pose2d> drivePoseSupplier) {
    return Commands.sequence(
        this.setState(PivotState.AIMING),
        run(
            () ->
                io.setTargetPosition(
                    Rotations.of(
                        PivotConstants.PIVOT_ANGLES_MAP_SIM.get(
                            drivePoseSupplier
                                .get()
                                .getTranslation()
                                .minus(
                                    FieldMirroringUtils.toCurrentAllianceTranslation(
                                            FieldMirroringUtils.SPEAKER_POSE_BLUE)
                                        .toTranslation2d())
                                .getNorm())))));
  }
}
