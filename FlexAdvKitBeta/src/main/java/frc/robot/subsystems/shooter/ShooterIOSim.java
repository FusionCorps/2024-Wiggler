package frc.robot.subsystems.shooter;

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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;

public class ShooterIOSim implements ShooterIO {
  DCMotorSim topShooterSim, bottomShooterSim;

  private final DCMotor SHOOTER_GEARBOX = DCMotor.getNeoVortex(1);

  private AngularVelocity topTargetVelocity = RadiansPerSecond.of(0);
  private AngularVelocity bottomTargetVelocity = RadiansPerSecond.of(0);

  private PIDController topController = new PIDController(0.03, 0.0, 0.0);
  private PIDController bottomController = new PIDController(0.03, 0.0, 0.0);
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.0, 0.017);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.0, 0.017);

  private ShooterConstants.ShooterState state = ShooterConstants.ShooterState.IDLE;
  Voltage voltsToApplyTop = Volts.of(0);
  Voltage voltsToApplyBottom = Volts.of(0);

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
    topTargetVelocity = inputs.shooterState.topRPM;
    bottomTargetVelocity = inputs.shooterState.bottomRPM;

    if (state == ShooterState.IDLE) {
      voltsToApplyTop = Volts.zero();
      voltsToApplyBottom = Volts.zero();
    } else {
      voltsToApplyTop =
          Volts.of(
              MathUtil.clamp(
                  topController.calculate(
                          topShooterSim.getAngularVelocityRadPerSec(),
                          topTargetVelocity.in(RadiansPerSecond))
                      + topFeedforward.calculate(topTargetVelocity.in(RadiansPerSecond)),
                  -12.0,
                  12.0));

      voltsToApplyBottom =
          Volts.of(
              MathUtil.clamp(
                  bottomController.calculate(
                          bottomShooterSim.getAngularVelocityRadPerSec(),
                          bottomTargetVelocity.in(RadiansPerSecond))
                      + bottomFeedforward.calculate(bottomTargetVelocity.in(RadiansPerSecond)),
                  -12.0,
                  12.0));
    }

    topShooterSim.setInputVoltage(voltsToApplyTop.in(Volts));
    bottomShooterSim.setInputVoltage(voltsToApplyBottom.in(Volts));

    topShooterSim.update(0.02);
    bottomShooterSim.update(0.02);

    inputs.topConnected = true;
    inputs.topShooterPositionRad = topShooterSim.getAngularPositionRad();
    inputs.topShooterVelocityRadPerSec = topShooterSim.getAngularVelocityRadPerSec();
    inputs.topShooterAppliedVolts = topShooterSim.getInputVoltage();
    inputs.topShooterCurrentAmps = topShooterSim.getCurrentDrawAmps();
    inputs.topVelocitySetpointRadPerSec = topTargetVelocity.in(RadiansPerSecond);

    inputs.bottomConnected = true;
    inputs.bottomShooterPositionRad = bottomShooterSim.getAngularPositionRad();
    inputs.bottomShooterVelocityRadPerSec = bottomShooterSim.getAngularVelocityRadPerSec();
    inputs.bottomShooterAppliedVolts = bottomShooterSim.getInputVoltage();
    inputs.bottomShooterCurrentAmps = bottomShooterSim.getCurrentDrawAmps();
    inputs.bottomVelocitySetpointRadPerSec = bottomTargetVelocity.in(RadiansPerSecond);

    inputs.shooterState = state;
  }

  @Override
  public void setShooterState(ShooterConstants.ShooterState state) {
    this.state = state;
  }
}
