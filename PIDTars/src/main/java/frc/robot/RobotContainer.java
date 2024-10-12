package frc.robot;

import static frc.robot.Constants.OperatorConstants.kDriverControllerPort;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  // Define the controller and robot subsystems
  private CommandXboxController controller =  new CommandXboxController(kDriverControllerPort);
  private Intake intake = new Intake();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /** Configure button bindings for the controller. */
  private void configureButtonBindings() {
    // // SysId button bindings
    // controller.a().onTrue(intake.sysIDQuasistatic(Direction.kForward));
    // controller.b().onTrue(intake.sysIDQuasistatic(Direction.kReverse));
    // controller.x().onTrue(intake.sysIDDynamic(Direction.kForward));
    // controller.y().onTrue(intake.sysIDDynamic(Direction.kReverse));

    // // starts and stops the CTRE signal logger
    // controller.start().onTrue(Commands.runOnce(() -> SignalLogger.start()));
    // controller.back().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
  }

  /** Use this method to define your autonomous command. */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
