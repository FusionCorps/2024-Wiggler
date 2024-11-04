package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // TODO: add intake motor - use REV CANSparkFlex, id 6
  CANSparkFlex intakeMotor = new CANSparkFlex(6, CANSparkFlex.MotorType.kBrushless);
  
  // TODO: configure intake motor
  public Intake() {}

  // TODO: implement intake Command
  // simply set the speed of the intake motor to 80% output (lazy output mode, no need to use setpoint)
  public Command runIntake() {
    return Commands.none();
  }
}
