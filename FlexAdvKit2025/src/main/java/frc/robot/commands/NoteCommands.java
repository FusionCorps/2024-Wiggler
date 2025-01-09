package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.IndexConstants.IndexState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PivotConstants.PivotState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Map;

public class NoteCommands {
  private final Shooter shooter;
  private final Pivot pivot;
  private final Intake intake;
  private final Index index;

  public NoteCommands(Shooter shooter, Pivot pivot, Intake intake, Index index) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.intake = intake;
    this.index = index;
  }

  public Command intakeNoteCmd() {
    return Commands.parallel(
        shooter.setState(ShooterState.IDLE),
        pivot.setState(
            Constants.currentMode == Mode.SIM ? PivotState.INTAKE_SIM : PivotState.INTAKE),
        intake.setState(IntakeState.INTAKE),
        index.setState(IndexState.INTAKE));
  }

  public Command turnOffIntakeCmd() {
    return Commands.sequence(intake.setState(IntakeState.IDLE), index.setState(IndexState.IDLE));
  }

  public Command extakeThruIntakeCmd() {
    return Commands.sequence(
        pivot.setState(
            Constants.currentMode == Mode.SIM ? PivotState.EXTAKE_SIM : PivotState.EXTAKE),
        intake.setState(IntakeState.EXTAKE),
        index.setState(IndexState.EXTAKE));
  }

  public Command extakeThruIntakeCmdEnd() {
    return Commands.parallel(intake.setState(IntakeState.IDLE), index.setState(IndexState.IDLE));
  }

  public Command extakeThruShooterCmd() {
    return Commands.sequence(
        shooter.setState(ShooterConstants.ShooterState.EXTAKE), index.setState(IndexState.EXTAKE));
  }

  public Command extakeThruShooterCmdEnd() {
    return Commands.parallel(
        shooter.setState(ShooterConstants.ShooterState.IDLE), index.setState(IndexState.IDLE));
  }

  public Command shootCmd() {
    return Commands.select(
        Map.ofEntries(
            Map.entry(PivotState.AMP, shooter.setState(ShooterState.AMP)),
            Map.entry(PivotState.AMP_SIM, shooter.setState(ShooterState.AMP)),
            Map.entry(PivotState.SHUTTLE, shooter.setState(ShooterState.SHUTTLE)),
            Map.entry(PivotState.SHUTTLE_SIM, shooter.setState(ShooterState.SHUTTLE)),
            Map.entry(PivotState.SUBWOOFER, shooter.setState(ShooterState.SPEAKER)),
            Map.entry(PivotState.SUBWOOFER_SIM, shooter.setState(ShooterState.SPEAKER)),
            Map.entry(PivotState.INTAKE, shooter.setState(ShooterState.EXTAKE)),
            Map.entry(PivotState.INTAKE_SIM, shooter.setState(ShooterState.EXTAKE)),
            Map.entry(PivotState.EXTAKE, shooter.setState(ShooterState.EXTAKE)),
            Map.entry(PivotState.MANUAL_OVERRIDE, shooter.setState(ShooterState.SPEAKER)),
            Map.entry(PivotState.AIMING, shooter.setState(ShooterState.SPEAKER))),
        pivot::getState);
  }

  public Command shootCmdEnd() {
    return Commands.sequence(
        index.setState(
            (pivot.getState() == PivotState.AMP || pivot.getState() == PivotState.AMP_SIM)
                ? IndexState.AMP
                : IndexState.SPEAKER),
        Commands.waitSeconds(1.0),
        index.setState(IndexState.IDLE),
        shooter.setState(ShooterState.IDLE));
  }
}
