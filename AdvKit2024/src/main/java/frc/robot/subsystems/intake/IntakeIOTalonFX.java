package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class IntakeIOTalonFX implements IntakeIO {
  public final TalonFX intakeMotor = new TalonFX(0);

  private StatusSignal<Double> intakePosRot = intakeMotor.getPosition();
  private StatusSignal<Double> intakeVelocityRotPerSec = intakeMotor.getVelocity();
  private StatusSignal<Double> intakeAppliedVolts = intakeMotor.getMotorVoltage();

  private VoltageOut voltageOut = new VoltageOut(0.0);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

public IntakeIOTalonFX() {
  intakeMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  BaseStatusSignal.setUpdateFrequencyForAll(
      60, 
      intakePosRot,
      intakeVelocityRotPerSec,
      intakeAppliedVolts);
      intakeMotor.optimizeBusUtilization();
}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      intakePosRot,
      intakeVelocityRotPerSec,
      intakeAppliedVolts);
    
      inputs.intakePosRad = Units.rotationsToRadians(intakePosRot.getValueAsDouble());
      inputs.intakeVelocityRadPerSec = Units.rotationsToRadians(intakeVelocityRotPerSec.getValueAsDouble());
      inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
  }

  @Override
  public void setIntakeVelocity(double velocityRotPerSec) {
    intakeMotor.setControl(velocityVoltage.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setControl(voltageOut.withOutput(volts));
  }
}
