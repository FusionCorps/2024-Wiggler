package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.PivotConstants.PIVOT_GEAR_RATIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
  private DCMotorSim pivotMainSim, pivotFollowerSim;

  ArmFeedforward pivotFeedforward = new ArmFeedforward(0.1, 0.05, 0.1);
  PIDController pivotController = new PIDController(0.1, 0.0, 0.0);

  private final DCMotor PIVOT_GEARBOX = DCMotor.getKrakenX60(1);

  Angle pivotTargetPosition = Radians.of(0.0);

  public PivotIOSim() {
    pivotMainSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(PIVOT_GEARBOX, 0.001, PIVOT_GEAR_RATIO),
            PIVOT_GEARBOX);
    pivotFollowerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(PIVOT_GEARBOX, 0.001, PIVOT_GEAR_RATIO),
            PIVOT_GEARBOX);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotMainConnected = true;
    inputs.pivotFollowerConnected = true;

    Voltage voltsToApply =
        Volts.of(
            pivotController.calculate(
                    pivotMainSim.getAngularPositionRad(), pivotTargetPosition.in(Radians))
                + pivotFeedforward.calculate(
                    pivotTargetPosition.in(Radians), pivotMainSim.getAngularVelocityRadPerSec()));

    pivotMainSim.setInputVoltage(MathUtil.clamp(voltsToApply.in(Volts), -12.0, 12.0));
    pivotFollowerSim.setInputVoltage(MathUtil.clamp(voltsToApply.in(Volts), -12.0, 12.0));

    pivotMainSim.update(0.02);
    pivotFollowerSim.update(0.02);

    inputs.pivotMainPositionRad = pivotMainSim.getAngularPositionRad();
    inputs.pivotMainVelocityRadPerSec = pivotMainSim.getAngularVelocityRadPerSec();
    inputs.pivotMainAppliedVolts = voltsToApply.in(Volts);
    inputs.pivotMainCurrentAmps = Math.abs(pivotMainSim.getCurrentDrawAmps());

    inputs.pivotFollowerPositionRad = pivotFollowerSim.getAngularPositionRad();
    inputs.pivotFollowerVelocityRadPerSec = pivotFollowerSim.getAngularVelocityRadPerSec();
    inputs.pivotFollowerAppliedVolts = voltsToApply.in(Volts);
    inputs.pivotFollowerCurrentAmps = Math.abs(pivotFollowerSim.getCurrentDrawAmps());

    inputs.pivotPositionSetpointRad = pivotTargetPosition.in(Radians);
  }

  @Override
  public void setPosition(Angle position) {
    pivotTargetPosition = position;
  }
}
