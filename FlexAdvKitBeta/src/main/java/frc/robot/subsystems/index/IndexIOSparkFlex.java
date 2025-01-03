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
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.IndexConstants.IndexState;
import java.util.function.DoubleSupplier;

public class IndexIOSparkFlex implements IndexIO {
  private final SparkFlex indexMotor = new SparkFlex(INDEX_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder indexEncoder = indexMotor.getEncoder();

  private final Debouncer indexMotorConnectedDebounce = new Debouncer(0.5);

  private IndexState state = IndexState.IDLE;

  public IndexIOSparkFlex() {
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(RobotController.getBatteryVoltage())
        .smartCurrentLimit(20);

    tryUntilOk(
        indexMotor,
        5,
        () ->
            indexMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    indexMotor.setVoltage(inputs.indexState.pct * 12.0);

    sparkStickyFault = false;
    ifOk(indexMotor, indexEncoder::getPosition, position -> inputs.indexPositionRad = position);
    ifOk(
        indexMotor,
        indexEncoder::getVelocity,
        velocity -> inputs.indexVelocityRadPerSec = velocity);
    ifOk(
        indexMotor,
        new DoubleSupplier[] {indexMotor::getAppliedOutput, indexMotor::getBusVoltage},
        (values) -> inputs.indexAppliedVolts = values[0] * values[1]);
    ifOk(indexMotor, indexMotor::getOutputCurrent, current -> inputs.indexCurrentAmps = current);
    inputs.indexMotorConnected = indexMotorConnectedDebounce.calculate(!sparkStickyFault);
    inputs.indexState = state;
  }

  @Override
  public void setIndexState(IndexState state) {
    this.state = state;
  }
}
