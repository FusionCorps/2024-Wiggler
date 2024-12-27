package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexIOSim implements IndexIO {
  DCMotorSim indexMotorSim;

  DCMotor INDEX_GEARBOX = DCMotor.getNeoVortex(1);

  private Voltage voltsToApply = Volts.of(0.0);

  public IndexIOSim() {
    indexMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEX_GEARBOX, 0.001, 1.0), INDEX_GEARBOX);
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    inputs.indexMotorConnected = true;

    indexMotorSim.setInputVoltage(MathUtil.clamp(voltsToApply.in(Volts), -12.0, 12.0));
    indexMotorSim.update(0.02);

    inputs.indexMotorPositionRad = indexMotorSim.getAngularPositionRad();
    inputs.indexMotorVelocityRadPerSec = indexMotorSim.getAngularVelocityRadPerSec();
    inputs.indexMotorAppliedVolts = voltsToApply.in(Volts);
    inputs.indexMotorCurrentAmps = Math.abs(indexMotorSim.getCurrentDrawAmps());
  }

  @Override
  public void setOutputVolts(Voltage volts) {
    voltsToApply = volts;
  }
}
