// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  Arm arm;
  CommandXboxController controller;

  double targetAngle = 0.0;
  double targetRPM = 0.0;

  @Override
  public void robotInit() {
    arm = new Arm();
    controller = new CommandXboxController(0);
    
    configureBindings();
  }

  private void configureBindings() {
    controller.povUp().whileTrue((Commands.run(() -> targetAngle = 0.0).alongWith(arm.setTargetPosition(() -> targetAngle))));
    controller.povDown().whileTrue((Commands.run(() -> targetAngle = 0.5).alongWith(arm.setTargetPosition(() -> targetAngle))));
    controller.povLeft().whileTrue((Commands.run(() -> targetAngle = 0.25).alongWith(arm.setTargetPosition(() -> targetAngle))));
    controller.povRight().whileTrue((Commands.run(() -> targetAngle = 0.75).alongWith(arm.setTargetPosition(() -> targetAngle))));

    // controller.povRight().whileTrue((Commands.run(() -> targetRPM += 1).alongWith(arm.setTargetVelocity(() -> targetRPM))).repeatedly());
    // controller.povLeft().whileTrue((Commands.run(() -> targetRPM -= 1).alongWith(arm.setTargetVelocity(() -> targetRPM))).repeatedly());

    controller.a().onTrue(arm.sysIDQuasistatic(Direction.kForward));
    controller.b().onTrue(arm.sysIDQuasistatic(Direction.kReverse));
    controller.x().onTrue(arm.sysIDDynamic(Direction.kForward));
    controller.y().onTrue(arm.sysIDDynamic(Direction.kReverse));

    controller.start().onTrue(Commands.runOnce(() -> SignalLogger.start()));
    controller.back().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
}
