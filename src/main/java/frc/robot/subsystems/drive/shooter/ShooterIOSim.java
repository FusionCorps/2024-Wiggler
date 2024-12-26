package frc.robot.subsystems.drive.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  DCMotorSim topShooterSim, bottomShooterSim;

  private final DCMotor SHOOTER_GEARBOX = DCMotor.getNeoVortex(1);

  private AngularVelocity topTargetVelocity = RadiansPerSecond.of(0);
  private AngularVelocity bottomTargetVelocity = RadiansPerSecond.of(0);

  private PIDController topController = new PIDController(0.1, 0.0, 0.0);
  private PIDController bottomController = new PIDController(0.1, 0.0, 0.0);
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.1, 0.2);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.1, 0.2);

  public ShooterIOSim() {
    topShooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(SHOOTER_GEARBOX, 0.001, 1.0), SHOOTER_GEARBOX);
    bottomShooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(SHOOTER_GEARBOX, 0.001, 1.0), SHOOTER_GEARBOX);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    Voltage voltsToApplyTop =
        Volts.of(
            topController.calculate(
                    topShooterSim.getAngularVelocityRadPerSec(),
                    topTargetVelocity.in(RadiansPerSecond))
                + topFeedforward.calculate(topTargetVelocity.in(RadiansPerSecond)));
    Voltage voltsToApplyBottom =
        Volts.of(
            bottomController.calculate(
                    bottomShooterSim.getAngularVelocityRadPerSec(),
                    bottomTargetVelocity.in(RadiansPerSecond))
                + bottomFeedforward.calculate(bottomTargetVelocity.in(RadiansPerSecond)));

    topShooterSim.setInputVoltage(MathUtil.clamp(voltsToApplyTop.in(Volts), -12.0, 12.0));
    bottomShooterSim.setInputVoltage(MathUtil.clamp(voltsToApplyBottom.in(Volts), -12.0, 12.0));

    topShooterSim.update(0.02);
    bottomShooterSim.update(0.02);

    inputs.topConnected = true;
    inputs.topShooterPositionRad = topShooterSim.getAngularPositionRad();
    inputs.topShooterVelocityRadPerSec = topShooterSim.getAngularVelocityRadPerSec();
    inputs.topShooterAppliedVolts = voltsToApplyTop.in(Volts);
    inputs.topShooterCurrentAmps = topShooterSim.getCurrentDrawAmps();

    inputs.bottomConnected = true;
    inputs.bottomShooterPositionRad = bottomShooterSim.getAngularPositionRad();
    inputs.bottomShooterVelocityRadPerSec = bottomShooterSim.getAngularVelocityRadPerSec();
    inputs.bottomShooterAppliedVolts = voltsToApplyBottom.in(Volts);
    inputs.bottomShooterCurrentAmps = bottomShooterSim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
    topTargetVelocity = topVelocity;
    bottomTargetVelocity = bottomVelocity;
  }
}
