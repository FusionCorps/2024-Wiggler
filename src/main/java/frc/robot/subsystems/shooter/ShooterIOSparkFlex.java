package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.DoubleSupplier;

public class ShooterIOSparkFlex implements ShooterIO {
  protected final SparkFlex topShooter = new SparkFlex(SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless);
  protected final SparkFlex bottomShooter =
      new SparkFlex(SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless);

  private final Debouncer topConnectedDebounce = new Debouncer(0.5);
  private final Debouncer bottomConnectedDebounce = new Debouncer(0.5);

  private final RelativeEncoder topEncoder = topShooter.getEncoder();
  private final RelativeEncoder bottomEncoder = bottomShooter.getEncoder();

  private final SparkClosedLoopController topController = topShooter.getClosedLoopController();
  private final SparkClosedLoopController bottomController =
      bottomShooter.getClosedLoopController();

  private double topTargetVelocity = 0;
  private double bottomTargetVelocity = 0;

  public ShooterIOSparkFlex() {
    SparkFlexConfig topConfig = new SparkFlexConfig();
    topConfig.idleMode(IdleMode.kCoast);
    topConfig.inverted(false);
    topConfig.smartCurrentLimit(
        ((int) SHOOTER_STALL_LIMIT_CURRENT.in(Amps)),
        (int) SHOOTER_FREE_LIMIT_CURRENT.in(Amps),
        (int) SHOOTER_FREE_SPEED_LIMIT.in(RPM));
    topConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kFF)
        .outputRange(-SHOOTER_MAX_RPM.in(RPM), SHOOTER_MAX_RPM.in(RPM));
    topConfig
        .encoder
        .positionConversionFactor((2 * Math.PI))
        .velocityConversionFactor((2 * Math.PI) / 60); // convert to rad and rad/sec

    SparkFlexConfig bottomConfig = new SparkFlexConfig();
    bottomConfig.apply(topConfig);
    bottomConfig.inverted(true);

    tryUntilOk(
        topShooter,
        5,
        () ->
            topShooter.configure(
                topConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    tryUntilOk(
        bottomShooter,
        5,
        () ->
            bottomShooter.configure(
                bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(topShooter, topEncoder::getPosition, position -> inputs.topShooterPositionRad = position);
    ifOk(
        topShooter,
        topEncoder::getVelocity,
        velocity -> inputs.topShooterVelocityRadPerSec = velocity);
    ifOk(
        topShooter,
        new DoubleSupplier[] {topShooter::getAppliedOutput, topShooter::getBusVoltage},
        (values) -> inputs.topShooterAppliedVolts = values[0] * values[1]);
    ifOk(
        topShooter,
        topShooter::getOutputCurrent,
        current -> inputs.topShooterCurrentAmps = current);
    inputs.topConnected = topConnectedDebounce.calculate(!sparkStickyFault);
    inputs.topShooterVelocityRadPerSec = topTargetVelocity;

    sparkStickyFault = false;
    ifOk(
        bottomShooter,
        bottomEncoder::getPosition,
        position -> inputs.bottomShooterPositionRad = position);
    ifOk(
        bottomShooter,
        bottomEncoder::getVelocity,
        velocity -> inputs.bottomShooterVelocityRadPerSec = velocity);
    ifOk(
        bottomShooter,
        new DoubleSupplier[] {bottomShooter::getAppliedOutput, bottomShooter::getBusVoltage},
        (values) -> inputs.bottomShooterAppliedVolts = values[0] * values[1]);
    ifOk(
        bottomShooter,
        bottomShooter::getOutputCurrent,
        current -> inputs.bottomShooterCurrentAmps = current);
    inputs.bottomConnected = bottomConnectedDebounce.calculate(!sparkStickyFault);
    inputs.bottomShooterVelocityRadPerSec = bottomTargetVelocity;
  }

  @Override
  public void setVelocity(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
    topTargetVelocity = topVelocity.in(RadiansPerSecond);
    bottomTargetVelocity = bottomVelocity.in(RadiansPerSecond);
    topController.setReference(topTargetVelocity, ControlType.kVelocity);
    bottomController.setReference(bottomTargetVelocity, ControlType.kVelocity);
  }
}
