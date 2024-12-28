package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.PivotConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOTalonFX implements PivotIO {
  private final TalonFX pivotMainMotor = new TalonFX(PIVOT_MAIN_MOTOR_ID);
  private final TalonFX pivotFollowerMotor = new TalonFX(PIVOT_FOLLOWER_MOTOR_ID);

  private final StatusSignal<Angle> pivotMainPosition;
  private final StatusSignal<AngularVelocity> pivotMainVelocity;
  private final StatusSignal<Voltage> pivotMainAppliedVoltage;
  private final StatusSignal<Current> pivotMainCurrent;

  private final StatusSignal<Angle> pivotFollowerPosition;
  private final StatusSignal<AngularVelocity> pivotFollowerVelocity;
  private final StatusSignal<Voltage> pivotFollowerAppliedVoltage;
  private final StatusSignal<Current> pivotFollowerCurrent;

  private final Debouncer pivotMainConnectedDebounce = new Debouncer(0.5);
  private final Debouncer pivotFollowerConnectedDebounce = new Debouncer(0.5);

  private final MotionMagicVoltage pivotMotionMagicPositionRequest = new MotionMagicVoltage(0);

  public PivotIOTalonFX() {
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConfig.Slot0.kV = PIVOT_kV;
    pivotConfig.Slot0.kP = PIVOT_kP;
    pivotConfig.Slot0.kI = PIVOT_kI;
    pivotConfig.Slot0.kD = PIVOT_kD;

    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = PIVOT_CRUISE_VELOCITY;
    pivotConfig.MotionMagic.MotionMagicAcceleration = PIVOT_ACCELERATION;
    pivotConfig.MotionMagic.MotionMagicJerk = PIVOT_JERK;

    tryUntilOk(5, () -> pivotMainMotor.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotFollowerMotor.getConfigurator().apply(pivotConfig, 0.25));
    pivotFollowerMotor.setControl(new Follower(PIVOT_MAIN_MOTOR_ID, false));

    pivotMainMotor.setPosition(0.0);
    pivotFollowerMotor.setPosition(0.0);

    pivotMainPosition = pivotMainMotor.getPosition();
    pivotMainVelocity = pivotMainMotor.getVelocity();
    pivotMainAppliedVoltage = pivotMainMotor.getMotorVoltage();
    pivotMainCurrent = pivotMainMotor.getStatorCurrent();

    pivotFollowerPosition = pivotFollowerMotor.getPosition();
    pivotFollowerVelocity = pivotFollowerMotor.getVelocity();
    pivotFollowerAppliedVoltage = pivotFollowerMotor.getMotorVoltage();
    pivotFollowerCurrent = pivotFollowerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50.0),
        pivotMainPosition,
        pivotMainVelocity,
        pivotMainAppliedVoltage,
        pivotMainCurrent,
        pivotFollowerPosition,
        pivotFollowerVelocity,
        pivotFollowerAppliedVoltage,
        pivotFollowerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(pivotMainMotor, pivotFollowerMotor);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusCode pivotMainStatus =
        BaseStatusSignal.refreshAll(
            pivotMainPosition, pivotMainVelocity, pivotMainAppliedVoltage, pivotMainCurrent);
    StatusCode pivotFollowerStatus =
        BaseStatusSignal.refreshAll(
            pivotFollowerPosition,
            pivotFollowerVelocity,
            pivotFollowerAppliedVoltage,
            pivotFollowerCurrent);
    inputs.pivotMainConnected = pivotMainConnectedDebounce.calculate(pivotMainStatus.isOK());
    inputs.pivotMainPositionRad = pivotMainPosition.getValue().in(Radians);
    inputs.pivotMainVelocityRadPerSec = pivotMainVelocity.getValue().in(RadiansPerSecond);
    inputs.pivotMainAppliedVolts = pivotMainAppliedVoltage.getValue().in(Volts);
    inputs.pivotMainCurrentAmps = pivotMainCurrent.getValue().in(Amps);

    inputs.pivotFollowerConnected =
        pivotFollowerConnectedDebounce.calculate(pivotFollowerStatus.isOK());
    inputs.pivotFollowerPositionRad = pivotFollowerPosition.getValue().in(Radians);
    inputs.pivotFollowerVelocityRadPerSec = pivotFollowerVelocity.getValue().in(RadiansPerSecond);
    inputs.pivotFollowerAppliedVolts = pivotFollowerAppliedVoltage.getValue().in(Volts);
    inputs.pivotFollowerCurrentAmps = pivotFollowerCurrent.getValue().in(Amps);

    inputs.pivotPositionSetpointRad =
        pivotMotionMagicPositionRequest.getPositionMeasure().in(Radians);
  }

  @Override
  public void setPosition(Angle position) {
    pivotMainMotor.setControl(pivotMotionMagicPositionRequest.withPosition(position));
  }
}
