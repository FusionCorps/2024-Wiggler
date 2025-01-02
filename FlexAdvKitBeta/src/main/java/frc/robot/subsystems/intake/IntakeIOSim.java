package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants.IntakeState;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim intakeMotorSim;
  private final DCMotor INTAKE_GEARBOX = DCMotor.getNeoVortex(1);

  private final IntakeSimulation intakePhysicsSim;

  private final BooleanSupplier isShooting;

  private Voltage appliedVolts = Volts.of(0.0);
  private IntakeState state = IntakeState.IDLE;

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrainSim, BooleanSupplier isShooting) {
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, 0.001, 1.0), INTAKE_GEARBOX);

    intakePhysicsSim =
        IntakeSimulation.InTheFrameIntake(
            "Note", driveTrainSim, Inches.of(23.5), IntakeSimulation.IntakeSide.FRONT, 1);
    this.isShooting = isShooting;
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

    inputs.beamBreakSensorConnected = true;
    inputs.noteInIntake = intakePhysicsSim.getGamePiecesAmount() != 0;
    inputs.intakeState = state;

    if (isShooting.getAsBoolean()) {
      intakePhysicsSim.obtainGamePieceFromIntake();
    }
  }

  @Override
  public void setOutputVolts(Voltage volts) {
    appliedVolts = volts;
    if (volts.equals(Volts.of(0.0))) {
      intakePhysicsSim.stopIntake();
    } else {
      intakePhysicsSim.startIntake();
    }
  }

  @Override
  public void setIntakeState(IntakeState state) {
    this.state = state;
  }
}
