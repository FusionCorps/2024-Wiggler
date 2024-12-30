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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotState;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
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
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOLimelight(CAM_0_NAME, drive::getRotation));

        intake = new Intake(new IntakeIOTalonFX());
        shooter = new Shooter(new ShooterIOSparkFlex());
        pivot = new Pivot(new PivotIOTalonFX());
        index = new Index(new IndexIOSparkFlex());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG, new Pose2d(3, 1.5, new Rotation2d()));
        Arena2024Crescendo.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(DriveConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(DriveConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(DriveConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(DriveConstants.BackRight, driveSimulation.getModules()[3]));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    CAM_0_NAME, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose));

        shooter = new Shooter(new ShooterIOSim());
        intake =
            new Intake(
                new IntakeIOSim(driveSimulation, () -> shooter.getState() != ShooterState.IDLE));
        pivot = new Pivot(new PivotIOSim());
        index =
            new Index(
                new IndexIOSim(
                    intake.noteInIntake,
                    driveSimulation::getSimulatedDriveTrainPose,
                    drive::getChassisSpeeds,
                    pivot::getPivotAngle,
                    Arena2024Crescendo.getInstance()));
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        pivot = new Pivot(new PivotIO() {});
        index = new Index(new IndexIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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

    // Center in-place on apriltag target
    controller.rightBumper().whileTrue(DriveCommands.centerOnTarget(drive, vision));

    // run intake mechanism while held
    controller
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                shooter.setVelocityState(ShooterState.IDLE),
                pivot.setPositionState(PivotState.INTAKE),
                intake.setVelocityState(IntakeState.INTAKE),
                index.setVelocityState(IndexIO.IndexState.INTAKE)))
        .onFalse(
            Commands.parallel(
                intake.setVelocityState(IntakeState.IDLE),
                index.setVelocityState(IndexIO.IndexState.IDLE)));
    // shut off intake mechanism when note enters intake
    intake.noteInIntake.onTrue(
        Commands.parallel(
            intake.setVelocityState(IntakeState.IDLE),
            index.setVelocityState(IndexIO.IndexState.IDLE)));

    // manual override pivot angle

    // idle shooter
    controller.y().onTrue(shooter.setVelocityState(ShooterState.IDLE));

    // move to preset pivot positions
    controller.a().onTrue(pivot.setPositionState(PivotState.SUBWOOFER));
    controller.leftBumper().onTrue(pivot.setPositionState(PivotState.AMP));
    controller.x().onTrue(pivot.setPositionState(PivotState.SHUTTLE));

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                () ->
                    driveSimulation.setSimulationWorldPose(new Pose2d(3, 1.5, new Rotation2d()))));

    // hold to rev shooter, let go to shoot
    controller
        .leftTrigger()
        .onTrue(shooter.setVelocityState(ShooterState.SPEAKER))
        .onFalse(
            index
                .setVelocityState(IndexIO.IndexState.SHOOT)
                .andThen(Commands.waitSeconds(1.0)) // account for delays in shooting
                .andThen(
                    index
                        .setVelocityState(IndexIO.IndexState.IDLE)
                        .alongWith(shooter.setVelocityState(ShooterState.IDLE))));

    // controller.povLeft()
    // controller.povRight()
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
    System.out.println("Resetting simulation field");
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
