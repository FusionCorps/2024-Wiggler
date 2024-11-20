package frc.robot;

import static frc.robot.Constants.kDriverControllerPort;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class Robot extends TimedRobot {
  // driver controller
  CommandXboxController driverController;

  // Drive Train subsystem
  private final DriveTrain driveTrain;

  // define autonomous command
  private Command autonomousCommand;

  public Robot() {
    driverController = new CommandXboxController(kDriverControllerPort);
    driveTrain = new DriveTrain();
    autonomousCommand = driveTrain.forwardAndBackward();
  }

  /** This function is where all controller button bindings should be configured. */
  public void configureButtonBindings() {
    driveTrain.setDefaultCommand(
        driveTrain.arcadeDrive(
            () -> -driverController.getLeftY(), () -> -driverController.getRightX()));
  }

  /* Period method that runs every ~20ms */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}
