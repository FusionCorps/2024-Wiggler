// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.SwerveConstants.DriveTrain;
import static frc.robot.Constants.SwerveConstants.MaxAngularRate;
import static frc.robot.Constants.SwerveConstants.MaxSpeed;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = DriveTrain; // My drivetrain
  private final Intake intake = new Intake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final Command auto = new PathPlannerAuto("TestAuto");

  // SPECIFY WHICH SYSID TO RUN HERE
  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.runSwerveFC(
            () -> -joystick.getLeftY() * MaxSpeed,
            () -> -joystick.getLeftX() * MaxSpeed,
            () -> -joystick.getRightX() * MaxAngularRate));
    
    joystick
        .b()
        .onTrue(
            drivetrain
                .runOnce(() -> drivetrain.seedFieldRelative())
                .alongWith(Commands.print("Gyro reset"))
                .withName("Reset Gyro"));
    joystick.a().whileTrue(intake.runIntake());

    joystick
        .back()
        .onTrue(Commands.runOnce(() -> SignalLogger.stop()).andThen(Commands.print("end")));
    joystick
        .start()
        .onTrue(Commands.runOnce(() -> SignalLogger.start()).andThen(Commands.print("start")));

    // SPECIFIC MODE IS DEFINED IN DRIVETRAIN SUBSYSTEM
    joystick.povUp().onTrue(drivetrain.runSysId(false, Direction.kForward));
    joystick.povDown().onTrue(drivetrain.runSysId(false, Direction.kReverse));
    joystick.povRight().onTrue(drivetrain.runSysId(true, Direction.kForward));
    joystick.povLeft().onTrue(drivetrain.runSysId(true, Direction.kReverse));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return auto;
  }
}
