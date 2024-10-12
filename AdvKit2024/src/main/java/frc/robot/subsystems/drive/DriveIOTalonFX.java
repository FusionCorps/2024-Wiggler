package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class DriveIOTalonFX implements DriveIO {
  TalonFX leftDriveMotor = new TalonFX(1);
  TalonFX rightDriveMotor = new TalonFX(2);

  VoltageOut leftVoltageOut = new VoltageOut(0.0);
  VoltageOut rightVoltageOut = new VoltageOut(0.0);

  VelocityVoltage leftVelocityVoltage = new VelocityVoltage(0.0);
  VelocityVoltage rightVelocityVoltage = new VelocityVoltage(0.0);

  private StatusSignal<Double> leftDriveMotorPosRad = leftDriveMotor.getPosition();
  private StatusSignal<Double> leftDriveMotorVelocityRadPerSec = leftDriveMotor.getVelocity();
  private StatusSignal<Double> leftDriveMotorAppliedVolts = leftDriveMotor.getMotorVoltage();
  
  private StatusSignal<Double> rightDriveMotorPosRad = rightDriveMotor.getPosition();
  private StatusSignal<Double> rightDriveMotorVelocityRadPerSec = rightDriveMotor.getVelocity();
  private StatusSignal<Double> rightDriveMotorAppliedVolts = rightDriveMotor.getMotorVoltage();

  public DriveIOTalonFX() {
    leftDriveMotor.setInverted(false);
    rightDriveMotor.setInverted(true);
    leftDriveMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    rightDriveMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    BaseStatusSignal.setUpdateFrequencyForAll(
      60,
      leftDriveMotorPosRad,
      leftDriveMotorVelocityRadPerSec,
      leftDriveMotorAppliedVolts,
      rightDriveMotorPosRad,
      rightDriveMotorVelocityRadPerSec,
      rightDriveMotorAppliedVolts);
    leftDriveMotor.optimizeBusUtilization();
    rightDriveMotor.optimizeBusUtilization();
  }

  @Override
  public void setDriveVelocity(double leftVelocityRadPerSec, double rightVelocityRadPerSec) {
    leftDriveMotor.setControl(leftVelocityVoltage.withVelocity(leftVelocityRadPerSec));
    rightDriveMotor.setControl(rightVelocityVoltage.withVelocity(rightVelocityRadPerSec));
  }

  @Override
  public void setDriveVoltage(double leftVolts, double rightVolts) {
    leftDriveMotor.setControl(leftVoltageOut.withOutput(leftVolts));
    rightDriveMotor.setControl(rightVoltageOut.withOutput(rightVolts));
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      leftDriveMotorPosRad,
      leftDriveMotorVelocityRadPerSec,
      leftDriveMotorAppliedVolts,
      rightDriveMotorPosRad,
      rightDriveMotorVelocityRadPerSec,
      rightDriveMotorAppliedVolts
    );

    inputs.leftDriveMotorAppliedVolts = leftDriveMotorAppliedVolts.getValueAsDouble();
    inputs.leftDriveMotorPosRad = Units.rotationsToRadians(leftDriveMotorPosRad.getValueAsDouble());
    inputs.leftDriveMotorVelocityRadPerSec = Units.rotationsToRadians(leftDriveMotorVelocityRadPerSec.getValueAsDouble());

    inputs.rightDriveMotorAppliedVolts = rightDriveMotorAppliedVolts.getValueAsDouble();
    inputs.rightDriveMotorPosRad = Units.rotationsToRadians(rightDriveMotorPosRad.getValueAsDouble());
    inputs.rightDriveMotorVelocityRadPerSec = Units.rotationsToRadians(rightDriveMotorVelocityRadPerSec.getValueAsDouble());
  }
}
