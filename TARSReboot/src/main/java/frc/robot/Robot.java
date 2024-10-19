// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OperatorConstants.kDriverControllerPort;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private final CommandXboxController controller;
  private final Drivetrain drtvetrain;

  public Robot() {
    controller = new CommandXboxController(kDriverControllerPort);
    drtvetrain = new Drivetrain();

    drtvetrain.setDefaultCommand(
        drtvetrain.runSwerveFC(
            () -> -controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
