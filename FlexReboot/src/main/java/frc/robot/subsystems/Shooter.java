package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  // TODO: add shooter motor - use REV CANSparkMax, ids 2 and 3

  // TODO: create objects for the PID controllers of the shooter motors, as we will use velocity PID control
  
  // TODO: configure BOTH shooter motors, they should be in a follower configuration (ensure that they rotate correctly)
  // max rpm is 6784 RPM, motors should coast
  public Shooter() {}

  // TODO: implement command to rev wheels, set the speed of the motors to 5000 and 3000 RPM
  public Command revWheels() {
    return Commands.none();
  }
}
