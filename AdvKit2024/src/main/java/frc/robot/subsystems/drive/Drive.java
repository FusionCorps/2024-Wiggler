package frc.robot.subsystems.drive;

import static frc.robot.Constants.driverController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  DriveIO driveIO;
  DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  @AutoLogOutput public Mechanism2d leftMechanism2d = new Mechanism2d(1, 1);
  private MechanismLigament2d leftMotorMechanismModifiable =
      leftMechanism2d
          .getRoot("leftMotor", 0.5, 0.5)
          .append(new MechanismLigament2d("leftMotorAngle", 0.25, 0));

  @AutoLogOutput public Mechanism2d rightMechanism2d = new Mechanism2d(1, 1);
  private MechanismLigament2d rightMotorMechanismModifiable =
      rightMechanism2d
          .getRoot("rightMotor", 0.5, 0.5)
          .append(new MechanismLigament2d("rightMotorAngle", 0.25, 0));

  public Drive(DriveIO driveIO) {
    this.driveIO = driveIO;

    setDefaultCommand(
        runArcadeDrive(() -> -driverController.getLeftY(), driverController::getRightX));
  }

  public void periodic() {
    driveIO.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    leftMotorMechanismModifiable.setAngle(Units.radiansToDegrees(inputs.leftDriveMotorPosRad));
    rightMotorMechanismModifiable.setAngle(Units.radiansToDegrees(inputs.rightDriveMotorPosRad));
  }

  public Command runArcadeDrive(DoubleSupplier speed, DoubleSupplier rotation) {
    return run(
        () ->
            driveIO.setDriveVoltage(
                speed.getAsDouble() + rotation.getAsDouble(),
                speed.getAsDouble() - rotation.getAsDouble()));
  }
}
