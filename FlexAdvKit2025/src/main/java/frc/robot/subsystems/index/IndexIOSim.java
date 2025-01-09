package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexConstants.IndexState.*;

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
import frc.robot.Constants.IndexConstants.IndexState;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

public class IndexIOSim implements IndexIO {
  private final DCMotorSim indexMotorSim;

  private final DCMotor INDEX_GEARBOX = DCMotor.getNeoVortex(1);

  private Voltage voltsToApply = Volts.of(0.0);

  private Trigger noteInIntake;

  private NoteOnFly noteOnFly;

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> chassisSpeedsFieldRelative;
  private double velocityRPM;
  private final Supplier<Angle> pivotAngle;
  private final SimulatedArena simulatedArena;

  private IndexState state = IndexState.IDLE;

  public IndexIOSim(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeedsFieldRelative,
      Supplier<Angle> pivotAngle,
      SimulatedArena simulatedArena) {
    indexMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEX_GEARBOX, 0.001, 1.0), INDEX_GEARBOX);

    this.noteInIntake = null;

    this.robotPose = robotPose;
    this.chassisSpeedsFieldRelative = chassisSpeedsFieldRelative;
    this.pivotAngle = pivotAngle;
    this.simulatedArena = simulatedArena;
  }

  public void setNoteInIntake(Trigger noteInIntake) {
    this.noteInIntake = noteInIntake;
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    indexMotorSim.setInputVoltage(MathUtil.clamp((inputs.indexState.pct * 12.0), -12.0, 12.0));
    indexMotorSim.update(0.02);

    inputs.indexMotorConnected = true;
    inputs.indexPositionRad = indexMotorSim.getAngularPositionRad();
    inputs.indexVelocityRadPerSec = indexMotorSim.getAngularVelocityRadPerSec();
    inputs.indexAppliedVolts = voltsToApply.in(Volts);
    inputs.indexCurrentAmps = Math.abs(indexMotorSim.getCurrentDrawAmps());
    inputs.indexState = state;

    if ((state == SPEAKER || state == EXTAKE || state == AMP) && noteInIntake.getAsBoolean()) {
      launchNoteWithTrajectory();
    }
  }

  public void launchNoteWithTrajectory() {
    if (state == SPEAKER) velocityRPM = 6000;
    else if (state == EXTAKE) velocityRPM = 500;
    else velocityRPM = -500;
    noteOnFly =
        new NoteOnFly(
            // Specify the position of the chassis when the note is launched
            robotPose.get().getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s
            // reference frame)
            new Translation2d(0.2, 0),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of
            // the projectile
            chassisSpeedsFieldRelative.get(),
            // The shooter facing direction is opposite the robot’s facing direction
            robotPose.get().getRotation().plus(Rotation2d.k180deg),
            // Initial height of the flying note
            0.7,
            // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000
            // RPM
            velocityRPM / 6000 * 16,
            // The angle at which the note is launched
            -pivotAngle.get().in(Radians));
    noteOnFly.enableBecomeNoteOnFieldAfterTouchGround();
    if (state == IndexState.SPEAKER) {
      noteOnFly.asSpeakerShotNote(() -> System.out.println("Hit speaker, +2 points!"));
    } else if (state == IndexState.AMP) {
      noteOnFly.asAmpShotNote(() -> System.out.println("Hit amp, +1 points!"));
    }
    noteOnFly
        // Configure callbacks to visualize the flight trajectory of the projectile
        .withProjectileTrajectoryDisplayCallBack(
        // Callback for when the note will eventually hit the target (if configured)
        (pose3ds) -> {
          Logger.recordOutput("Index/NoteProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new));
        },
        // Callback for when the note will eventually miss the target, or if no target is configured
        (pose3ds) -> {
          Logger.recordOutput(
              "Index/NoteProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new));
        });
    if (noteInIntake.getAsBoolean()) {
      simulatedArena.addGamePieceProjectile(noteOnFly);
    }
  }

  @Override
  public void setIndexState(IndexState state) {
    this.state = state;
  }
}
