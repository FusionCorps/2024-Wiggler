package frc.robot.subsystems.index;

import static frc.robot.Constants.IndexConstants.*;
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
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.DoubleSupplier;

public class IndexIOSparkFlex implements IndexIO {
  SparkFlex indexMotor = new SparkFlex(INDEX_MOTOR_ID, MotorType.kBrushless);
  RelativeEncoder indexEncoder = indexMotor.getEncoder();

  Debouncer indexMotorConnectedDebounce = new Debouncer(0.5);

  public IndexIOSparkFlex() {
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(RobotController.getBatteryVoltage());

    tryUntilOk(
        indexMotor,
        5,
        () ->
            indexMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        indexMotor, indexEncoder::getPosition, position -> inputs.indexMotorPositionRad = position);
    ifOk(
        indexMotor,
        indexEncoder::getVelocity,
        velocity -> inputs.indexMotorVelocityRadPerSec = velocity);
    ifOk(
        indexMotor,
        new DoubleSupplier[] {indexMotor::getAppliedOutput, indexMotor::getBusVoltage},
        (values) -> inputs.indexMotorAppliedVolts = values[0] * values[1]);
    ifOk(
        indexMotor,
        indexMotor::getOutputCurrent,
        current -> inputs.indexMotorCurrentAmps = current);
    inputs.indexMotorConnected = indexMotorConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setOutputVolts(Voltage volts) {
    indexMotor.setVoltage(volts);
  }
}
