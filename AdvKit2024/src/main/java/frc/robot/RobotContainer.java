// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;

public class RobotContainer {
  private final Intake intake;
  private final Drive drive;
  
  public RobotContainer() {
    intake = new Intake(new IntakeIOSim());
    drive = new Drive(new DriveIOSim());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
