package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.BEAM_BREAK_SENSOR_ID;
import static frc.robot.Constants.IntakeConstants.INTAKE_MOTOR_ID;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.IntakeConstants.IntakeState;
import java.util.function.DoubleSupplier;

public class IntakeIOSparkFlex implements IntakeIO {
  private final SparkFlex intakeMotor = new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private DigitalInput beamBreakSensor;

  Debouncer intakeMotorConnectedDebounce = new Debouncer(0.5);

  private IntakeState state = IntakeState.IDLE;

  public IntakeIOSparkFlex() {
    try {
      beamBreakSensor = new DigitalInput(BEAM_BREAK_SENSOR_ID);
    } catch (Exception e) {
      beamBreakSensor = null;
    }

    SparkFlexConfig config = new SparkFlexConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(RobotController.getBatteryVoltage())
        .smartCurrentLimit(80);

    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(intakeMotor, intakeEncoder::getPosition, position -> inputs.intakePositionRad = position);
    ifOk(
        intakeMotor,
        intakeEncoder::getVelocity,
        velocity -> inputs.intakeVelocityRadPerSec = velocity);
    ifOk(
        intakeMotor,
        new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
        (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
    ifOk(intakeMotor, intakeMotor::getOutputCurrent, current -> inputs.intakeCurrentAmps = current);
    inputs.intakeMotorConnected = intakeMotorConnectedDebounce.calculate(!sparkStickyFault);

    inputs.beamBreakSensorConnected = beamBreakSensor != null;
    inputs.noteInIntake = beamBreakSensor != null && !beamBreakSensor.get();

    inputs.intakeState = state;
  }

  @Override
  public void setOutputVolts(Voltage volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setIntakeState(IntakeState state) {
    this.state = state;
  }
}
