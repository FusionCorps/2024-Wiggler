// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexConstants.IndexState;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PivotConstants.PivotState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.NoteCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.module.ModuleIOTalonFXSim;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexIO;
import frc.robot.subsystems.index.IndexIOSim;
import frc.robot.subsystems.index.IndexIOSparkFlex;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkFlex;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Shooter shooter;
  private final Pivot pivot;
  private final Index index;

  private SwerveDriveSimulation driveSimulation = null;

  private final Autos autos;
  private final NoteCommands noteCommands;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(DriveConstants.FrontLeft),
                new ModuleIOTalonFXReal(DriveConstants.FrontRight),
                new ModuleIOTalonFXReal(DriveConstants.BackLeft),
                new ModuleIOTalonFXReal(DriveConstants.BackRight));
        vision = new Vision(drive, new VisionIOLimelight(CAM_0_NAME, drive::getRotation));

        intake = new Intake(new IntakeIOSparkFlex());
        shooter = new Shooter(new ShooterIOSparkFlex());
        pivot = new Pivot(new PivotIOTalonFX());
        index = new Index(new IndexIOSparkFlex());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.MAPLE_SIM_CONFIG, new Pose2d(3, 1.5, new Rotation2d()));
        Arena2024Crescendo.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(DriveConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(DriveConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(DriveConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(DriveConstants.BackRight, driveSimulation.getModules()[3]));
        vision = null;
        new Vision(
            drive,
            new VisionIOPhotonVisionSim(
                CAM_1_NAME, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        shooter = new Shooter(new ShooterIOSim());
        pivot = new Pivot(new PivotIOSim());
        index =
            new Index(
                new IndexIOSim(
                    drive::getPose,
                    drive::getChassisSpeeds,
                    pivot::getPosition,
                    Arena2024Crescendo.getInstance()));
        intake =
            new Intake(
                new IntakeIOSim(
                    driveSimulation,
                    () -> {
                      return index.getIndexState() == IndexState.SPEAKER
                          || index.getIndexState() == IndexState.EXTAKE
                          || index.getIndexState() == IndexState.AMP;
                    }));
        index.setNoteInIntake(intake.noteInIntake);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        pivot = new Pivot(new PivotIO() {});
        index = new Index(new IndexIO() {});
        break;
    }

    autos = new Autos(drive, driveSimulation, intake, shooter, index, pivot);
    noteCommands = new NoteCommands(shooter, pivot, intake, index);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addOption("Three Piece", autos.ThreePieceAuto());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    // TODO: note that to switch to steer and rotation, runCharacterization should be modified in
    // Module.java
    autoChooser.addOption(
        "SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset to actual pose during simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.b().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // Center in-place on apriltag target and adjust pivot angle based on distance from speaker
    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                DriveCommands.centerOnSpeakerTag(
                    drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
                pivot.angleToSpeakerCommand(drive::getPose)));

    // run intake mechanism while held
    controller
        .rightTrigger()
        .whileTrue(noteCommands.intakeNoteCmd())
        .onFalse(noteCommands.turnOffIntakeCmd());
    // shut off intake mechanism automatically when note enters intake
    intake.noteInIntake.and(RobotModeTriggers.teleop()).whileTrue(noteCommands.turnOffIntakeCmd());

    // manually override pivot angle
    controller.povUp().whileTrue(pivot.setPivotPctCommand(0.05));

    controller.povDown().whileTrue(pivot.setPivotPctCommand(-0.05));

    // extake thru intake
    controller
        .povLeft()
        .whileTrue(noteCommands.extakeThruIntakeCmd())
        .onFalse(noteCommands.extakeThruIntakeCmdEnd());

    // extake thru shooter
    controller
        .povRight()
        .whileTrue(noteCommands.extakeThruShooterCmd())
        .onFalse(noteCommands.extakeThruShooterCmdEnd());

    // turn off shooter manually
    controller.y().onTrue(shooter.setState(ShooterState.IDLE));

    // move to preset pivot positions
    if (Constants.currentMode == Mode.SIM) {
      controller.a().onTrue(pivot.setState(PivotState.SUBWOOFER_SIM));
      controller.leftBumper().onTrue(pivot.setState(PivotState.AMP_SIM));
      controller.x().onTrue(pivot.setState(PivotState.SHUTTLE_SIM));
    } else {
      controller.a().onTrue(pivot.setState(PivotState.SUBWOOFER));
      controller.leftBumper().onTrue(pivot.setState(PivotState.AMP));
      controller.x().onTrue(pivot.setState(PivotState.SHUTTLE));
    }

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                () ->
                    driveSimulation.setSimulationWorldPose(new Pose2d(3, 1.5, new Rotation2d()))));

    // hold to rev shooter, let go to shoot
    controller.leftTrigger().whileTrue(noteCommands.shootCmd()).onFalse(noteCommands.shootCmdEnd());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 1.5, new Rotation2d()));
    // System.out.println("Resetting simulation field");
    Arena2024Crescendo.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Notes", Arena2024Crescendo.getInstance().getGamePiecesArrayByType("Note"));
  }
}
