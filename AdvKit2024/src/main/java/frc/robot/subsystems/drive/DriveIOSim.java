package frc.robot.subsystems.drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DriveIOSim implements DriveIO {
  DCMotorSim leftDriveMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.0001);
  DCMotorSim rightDriveMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.0001);

  private double leftDriveMotorAppliedVolts = 0.0;
  private double rightDriveMotorAppliedVolts = 0.0;

  public void setDriveVelocity(double leftVelocityRadPerSec, double rightVelocityRadPerSec) {
    leftDriveMotorSim.setInputVoltage(leftVelocityRadPerSec);
    rightDriveMotorSim.setInputVoltage(rightVelocityRadPerSec);
  }

  public void setDriveVoltage(double leftVolts, double rightVolts) {
    leftDriveMotorAppliedVolts = leftVolts;
    rightDriveMotorAppliedVolts = rightVolts;
    leftDriveMotorSim.setInputVoltage(leftVolts);
    rightDriveMotorSim.setInputVoltage(rightVolts);
  }

  public void updateInputs(DriveIO.DriveIOInputs inputs) {
    leftDriveMotorSim.update(0.02);
    rightDriveMotorSim.update(0.02);

    inputs.leftDriveMotorAppliedVolts = leftDriveMotorAppliedVolts;
    inputs.leftDriveMotorPosRad = leftDriveMotorSim.getAngularPositionRad();
    inputs.leftDriveMotorVelocityRadPerSec = leftDriveMotorSim.getAngularVelocityRadPerSec();

    inputs.rightDriveMotorAppliedVolts = rightDriveMotorAppliedVolts;
    inputs.rightDriveMotorPosRad = rightDriveMotorSim.getAngularPositionRad();
    inputs.rightDriveMotorVelocityRadPerSec = rightDriveMotorSim.getAngularVelocityRadPerSec();
  }
}
