package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  DCMotorSim intakeMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.0001);

  private double intakeAppliedVolts = 0.0;
  PIDController intakeController = new PIDController(1.0, 0.0, 0.0);
  SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  @Override
  public void setIntakeVelocity(double velocityRadPerSec) {
    intakeMotorSim.setInputVoltage(
        intakeFeedforward.calculate(velocityRadPerSec)
            + intakeController.calculate(
                intakeMotorSim.getAngularVelocityRadPerSec(), velocityRadPerSec));
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeMotorSim.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSim.update(0.02);
    inputs.intakeAppliedVolts = intakeAppliedVolts;
    inputs.intakePosRad = intakeMotorSim.getAngularPositionRad();
    inputs.intakeVelocityRadPerSec = intakeMotorSim.getAngularVelocityRadPerSec();
  }
}
