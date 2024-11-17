package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    /**
     * Xbox Controller for controlling robot *
     */
    CommandXboxController m_controller = new CommandXboxController(0); // No constantize

    /**
     * Drive Train (Subsystem) *
     */
    DriveTrain m_driveTrain = new DriveTrain();

    @Override
    public void robotInit() {
        m_driveTrain.setDefaultCommand(
                m_driveTrain.runArcadeDrive(
                        () -> -m_controller.getLeftY(),
                        () -> -m_controller.getRightX()));

        // While the A button is pressed, run the drive at 50% speed
        m_controller.a().whileTrue(m_driveTrain.runArcadeDrive(() -> 0.5, () -> 0.0));

        // While the A AND B button are pressed, run the drive at 100% speed
        m_controller.a().and(m_controller.b()).whileTrue(m_driveTrain.runArcadeDrive(() -> 1.0, () -> 0.0));
    }

    /**
     * Period method that runs every ~20ms
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }
}
