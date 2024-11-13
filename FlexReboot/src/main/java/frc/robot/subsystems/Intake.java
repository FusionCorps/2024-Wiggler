package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkFlex intakeMotor =
      new CANSparkFlex(Constants.kCANMotorID, CANSparkFlex.MotorType.kBrushless);

  public Intake() {
    /* Configure smart current limit  */
    intakeMotor.setSmartCurrentLimit(80);
    intakeMotor.burnFlash();
  }

  // simply set the speed of the intake motor to 80% output (lazy output mode, no need to use
  // setpoint)
  public Command runIntake() {
    return Commands.run(
        () -> {
          intakeMotor.set(0.8);
        });
  }
}
