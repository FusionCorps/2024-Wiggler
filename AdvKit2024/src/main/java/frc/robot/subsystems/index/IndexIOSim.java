package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.index.IndexIO.IndexState.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

public class IndexIOSim implements IndexIO {
  DCMotorSim indexMotorSim;

  DCMotor INDEX_GEARBOX = DCMotor.getNeoVortex(1);

  private Voltage voltsToApply = Volts.of(0.0);

  Trigger noteInIntake;

  private NoteOnFly noteOnFly;

  Supplier<Pose2d> robotSimulationWorldPose;
  Supplier<ChassisSpeeds> chassisSpeedsFieldRelative;
  double velocityRPM;
  Supplier<Angle> pivotAngle;
  SimulatedArena simulatedArena;

  private IndexState state = IndexState.IDLE;

  public IndexIOSim(
      Trigger noteInIntake,
      Supplier<Pose2d> robotSimulationWorldPose,
      Supplier<ChassisSpeeds> chassisSpeedsFieldRelative,
      Supplier<Angle> pivotAngle,
      SimulatedArena simulatedArena) {
    indexMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEX_GEARBOX, 0.001, 1.0), INDEX_GEARBOX);

    this.noteInIntake = noteInIntake;

    this.robotSimulationWorldPose = robotSimulationWorldPose;
    this.chassisSpeedsFieldRelative = chassisSpeedsFieldRelative;
    this.pivotAngle = pivotAngle;
    this.simulatedArena = simulatedArena;
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    inputs.indexMotorConnected = true;

    indexMotorSim.setInputVoltage(MathUtil.clamp(voltsToApply.in(Volts), -12.0, 12.0));
    indexMotorSim.update(0.02);

    inputs.indexMotorPositionRad = indexMotorSim.getAngularPositionRad();
    inputs.indexMotorVelocityRadPerSec = indexMotorSim.getAngularVelocityRadPerSec();
    inputs.indexMotorAppliedVolts = voltsToApply.in(Volts);
    inputs.indexMotorCurrentAmps = Math.abs(indexMotorSim.getCurrentDrawAmps());
    inputs.indexState = state;

    if ((state == SHOOT || state == EXTAKE || state == AMP) && noteInIntake.getAsBoolean()) {
      launchNoteWithTrajectory();
    }
  }

  @Override
  public void setOutputVolts(Voltage volts) {
    voltsToApply = volts;
  }

  public void launchNoteWithTrajectory() {
    if (state == SHOOT) velocityRPM = 6000;
    else if (state == EXTAKE) velocityRPM = 2000;
    else velocityRPM = -500;
    noteOnFly =
        new NoteOnFly(
            // Specify the position of the chassis when the note is launched
            robotSimulationWorldPose.get().getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s
            // reference frame)
            new Translation2d(0.2, 0),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of
            // the projectile
            chassisSpeedsFieldRelative.get(),
            // The shooter facing direction is the same as the robot’s facing direction
            robotSimulationWorldPose.get().getRotation().plus(Rotation2d.k180deg),
            // Initial height of the flying note
            0.7,
            // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000
            // RPM
            velocityRPM / 6000 * 16,
            // The angle at which the note is launched
            pivotAngle.get().in(Radians));
    simulatedArena.addGamePieceProjectile(noteOnFly);
    noteOnFly.enableBecomeNoteOnFieldAfterTouchGround();
    noteOnFly
        // Configure callbacks to visualize the flight trajectory of the projectile
        .withProjectileTrajectoryDisplayCallBack(
        // Callback for when the note will eventually hit the target (if configured)
        (pose3ds) -> {
          Logger.recordOutput("Index/NoteProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new));
          System.out.println("asdkjajbdhsadg");
        },
        // Callback for when the note will eventually miss the target, or if no target is configured
        (pose3ds) -> {
          Logger.recordOutput(
              "Index/NoteProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new));
          System.out.println("asdsadsada");
        });
    if (noteInIntake.getAsBoolean()) {
      noteOnFly.launch();
    }
  }

  @Override
  public void setIndexState(IndexState state) {
    this.state = state;
  }
}
