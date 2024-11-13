package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkFlex intakeMotor =
      new CANSparkFlex(Constants.kCANMotorID, CANSparkFlex.MotorType.kBrushless);

  public Intake() {
    /* Configure smart current limit  */
    intakeMotor.setSmartCurrentLimit(80);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.burnFlash();
  }

  // set the speed of the intake motor to 80% output when running
  public Command runIntake() {
    return runEnd(() -> intakeMotor.set(0.8), () -> intakeMotor.set(0));
  }
}
