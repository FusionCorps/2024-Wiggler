// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxSpeed =
      DrivetrainConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain =
      DrivetrainConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.runSwerveFC(
            () -> -joystick.getLeftY() * MaxSpeed,
            () -> -joystick.getLeftX() * MaxSpeed,
            () -> joystick.getRightX() * MaxAngularRate));
    joystick
        .b()
        .onTrue(
            drivetrain
                .runOnce(() -> drivetrain.seedFieldRelative())
                .alongWith(Commands.print("Gyro reset"))
                .withName("Reset Gyro"));

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
    return Commands.print("No autonomous command configured");
  }
}
