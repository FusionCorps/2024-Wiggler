package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexConstants.IndexState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LocalADStarAK;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Index index;
  private final Pivot pivot;
  private final SwerveDriveSimulation drivesim;

  Command forwardCmd;
  NoteCommands noteCommands;

  public Autos(
      Drive drive,
      SwerveDriveSimulation driveSim,
      Intake intake,
      Shooter shooter,
      Index index,
      Pivot pivot) {
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.index = index;
    this.pivot = pivot;
    this.drivesim = driveSim;

    // configure PathPlanner
    AutoBuilder.configure(
        drive::getPose,
        pose -> {
          drive.setPose(pose);
          if (driveSim != null) driveSim.setSimulationWorldPose(pose);
        },
        drive::getChassisSpeeds,
        drive::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(10.0, 0.0, 0.0), new PIDConstants(10.0, 0.0, 0.0)),
        DriveConstants.PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    noteCommands = new NoteCommands(shooter, pivot, intake, index);
  }

  public Command ThreePieceAuto() {
    PathPlannerPath[] pathSections = new PathPlannerPath[3];
    try {
      for (int i = 0; i < 3; i++) {
        pathSections[i] = PathPlannerPath.fromChoreoTrajectory("3Piece", i);
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
    return Commands.sequence(
        drive.runOnce(
            () -> {
              drive.setPose(pathSections[0].getStartingDifferentialPose());
              drivesim.setSimulationWorldPose(pathSections[0].getStartingDifferentialPose());
            }),
        // aimAndShoot(),
        followPathAndIntake(pathSections[0]),
        aimAndShoot(),
        followPathAndIntake(pathSections[1]),
        aimAndShoot(),
        followPathAndIntake(pathSections[2]),
        aimAndShoot());
  }

  private Command aimAndShoot() {
    return Commands.parallel(
            shooter.setState(ShooterState.SPEAKER),
            pivot.angleToSpeakerCommand(drive::getPose),
            DriveCommands.centerOnSpeakerTag(drive, () -> 0.0, () -> 0.0))
        .until(() -> shooter.atSpeed.and(pivot.atAngle).getAsBoolean())
        .andThen(index.setState(IndexState.SPEAKER));
  }

  private Command followPathAndIntake(PathPlannerPath path) {
    return Commands.sequence(
        shooter.setState(ShooterState.IDLE),
        noteCommands.intakeNoteCmd(),
        AutoBuilder.followPath(path),
        noteCommands.turnOffIntakeCmd());
  }
}
