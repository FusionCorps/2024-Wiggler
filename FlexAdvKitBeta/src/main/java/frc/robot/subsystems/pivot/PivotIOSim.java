package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.PivotConstants.PIVOT_GEAR_RATIO;
import static frc.robot.Constants.PivotConstants.PIVOT_kD;
import static frc.robot.Constants.PivotConstants.PIVOT_kI;
import static frc.robot.Constants.PivotConstants.PIVOT_kP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.PivotConstants.PivotState;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class PivotIOSim implements PivotIO {
  private final DCMotorSim pivotSim;
  private final DCMotor PIVOT_GEARBOX = DCMotor.getKrakenX60Foc(2);

  private final ProfiledPIDController pivotController =
      new ProfiledPIDController(
          PIVOT_kP, PIVOT_kI, PIVOT_kD, new TrapezoidProfile.Constraints(2500, 2000));

  private Angle pivotTargetPosition = Radians.of(0.0);
  private PivotState state = PivotState.INTAKE_SIM;

  private Voltage voltsToApply = Volts.of(0.0);

  public PivotIOSim() {
    pivotSim =
        new DCMotorSim(
            LinearSystemId.createSingleJointedArmSystem(PIVOT_GEARBOX, 0.001, PIVOT_GEAR_RATIO),
            PIVOT_GEARBOX);
    pivotSim.setAngle(0.0);
    pivotController.reset(0.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (state != PivotState.MANUAL_OVERRIDE && state != PivotState.AIMING)
      pivotTargetPosition = state.angle;

    inputs.pivotMainConnected = true;
    inputs.pivotFollowerConnected = true;

    // run closed-loop if not in manual override
    if (state != PivotState.MANUAL_OVERRIDE) {
      voltsToApply =
          Volts.of(
              MathUtil.clamp(
                  pivotController.calculate(
                      pivotSim.getAngularPositionRad(), pivotTargetPosition.in(Radians)),
                  -12.0,
                  12.0));
    }
    pivotSim.setInputVoltage(voltsToApply.in(Volts));
    pivotSim.update(0.02);

    inputs.pivotMainPositionRad = pivotSim.getAngularPositionRad();
    inputs.pivotMainVelocityRadPerSec = pivotSim.getAngularVelocityRadPerSec();
    inputs.pivotMainAppliedVolts = pivotSim.getInputVoltage();
    inputs.pivotMainCurrentAmps = Math.abs(pivotSim.getCurrentDrawAmps());

    inputs.pivotFollowerPositionRad = inputs.pivotMainPositionRad;
    inputs.pivotFollowerVelocityRadPerSec = inputs.pivotMainVelocityRadPerSec;
    inputs.pivotFollowerAppliedVolts = inputs.pivotMainAppliedVolts;
    inputs.pivotFollowerCurrentAmps = inputs.pivotMainCurrentAmps;

    inputs.pivotPositionSetpointRad = pivotTargetPosition.in(Radians);
    inputs.pivotState = state;
  }

  @Override
  public void setState(PivotState state) {
    this.state = state;
  }

  @Override
  public void setPct(double pct) {
    voltsToApply = Volts.of(pct * SimulatedBattery.getBatteryVoltage().in(Volts));
  }

  @Override
  public void setTargetPosition(Angle position) {
    pivotTargetPosition = position;
  }
}
