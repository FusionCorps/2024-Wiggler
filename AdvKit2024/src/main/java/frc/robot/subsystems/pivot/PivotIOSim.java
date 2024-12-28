package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.PivotConstants.PIVOT_GEAR_RATIO;
import static frc.robot.Constants.PivotConstants.PIVOT_kD;
import static frc.robot.Constants.PivotConstants.PIVOT_kI;
import static frc.robot.Constants.PivotConstants.PIVOT_kP;
import static frc.robot.Constants.PivotConstants.PIVOT_kV;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
  private DCMotorSim pivotSim;

  ArmFeedforward pivotFeedforward = new ArmFeedforward(0.0, 0.0, PIVOT_kV);
  PIDController pivotController = new PIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD);

  private final DCMotor PIVOT_GEARBOX = DCMotor.getKrakenX60(2);

  Angle pivotTargetPosition = Radians.of(0.0);

  public PivotIOSim() {
    pivotSim =
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
                    pivotSim.getAngularPositionRad(), pivotTargetPosition.in(Radians)))
            .plus(
                pivotFeedforward.calculate(
                    pivotTargetPosition,
                    RadiansPerSecond.of(pivotSim.getAngularVelocityRadPerSec())));

    pivotSim.setInputVoltage(MathUtil.clamp(voltsToApply.in(Volts), -12.0, 12.0));

    pivotSim.update(0.02);

    inputs.pivotMainPositionRad = pivotSim.getAngularPositionRad();
    inputs.pivotMainVelocityRadPerSec = pivotSim.getAngularVelocityRadPerSec();
    inputs.pivotMainAppliedVolts = voltsToApply.in(Volts);
    inputs.pivotMainCurrentAmps = Math.abs(pivotSim.getCurrentDrawAmps());

    inputs.pivotFollowerPositionRad = inputs.pivotMainPositionRad;
    inputs.pivotFollowerVelocityRadPerSec = inputs.pivotMainVelocityRadPerSec;
    inputs.pivotFollowerAppliedVolts = voltsToApply.in(Volts);
    inputs.pivotFollowerCurrentAmps = inputs.pivotMainCurrentAmps;

    inputs.pivotPositionSetpointRad = pivotTargetPosition.in(Radians);
  }

  @Override
  public void setPosition(Angle position) {
    pivotTargetPosition = position;
  }
}
