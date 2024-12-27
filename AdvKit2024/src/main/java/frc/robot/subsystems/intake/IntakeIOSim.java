package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeMotorSim;

  private Voltage appliedVolts = Volts.of(0.0);

  private final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60(1);

  public IntakeIOSim() {
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, 0.001, 1.0), INTAKE_GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorConnected = true;

    // Update motor sim
    intakeMotorSim.setInputVoltage(MathUtil.clamp(appliedVolts.in(Volts), -12.0, 12.0));
    intakeMotorSim.update(0.02);

    inputs.intakePositionRad = intakeMotorSim.getAngularPositionRad();
    inputs.intakeVelocityRadPerSec = intakeMotorSim.getAngularVelocityRadPerSec();
    inputs.intakeAppliedVolts = appliedVolts.in(Volts);
    inputs.intakeCurrentAmps = Math.abs(intakeMotorSim.getCurrentDrawAmps());
  }

  @Override
  public void setOutputVolts(Voltage volts) {
    appliedVolts = volts;
  }
}
