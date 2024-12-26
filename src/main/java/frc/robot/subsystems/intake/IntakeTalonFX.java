package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

public class IntakeTalonFX implements IntakeIO {
  TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVoltage;
  private final StatusSignal<Current> intakeCurrent;

  private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);

  private final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0);

  public IntakeTalonFX() {
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 80;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));

    intakePosition = intakeMotor.getPosition();
    intakeVelocity = intakeMotor.getVelocity();
    intakeAppliedVoltage = intakeMotor.getMotorVoltage();
    intakeCurrent = intakeMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50.0), intakePosition, intakeVelocity, intakeAppliedVoltage, intakeCurrent);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var intakeStatus =
        BaseStatusSignal.refreshAll(
            intakePosition, intakeVelocity, intakeAppliedVoltage, intakeCurrent);
    inputs.connected = intakeConnectedDebounce.calculate(intakeStatus.isOK());
    inputs.intakePositionRad = intakePosition.getValue().in(Radians);
    inputs.intakeVelocityRadPerSec = intakeVelocity.getValue().in(RadiansPerSecond);
    inputs.intakeAppliedVolts = intakeAppliedVoltage.getValue().in(Volts);
    inputs.intakeCurrentAmps = intakeCurrent.getValue().in(Amps);
  }

  @Override
  public void setOutputPercent(double outputPercent) {
    intakeMotor.setControl(intakeDutyCycle.withOutput(outputPercent));
  }
}
