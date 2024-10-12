// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor = new TalonFX(kIntakeMotorPort); // intake motor

  VelocityVoltage velocityRequest = new VelocityVoltage(0.0); // velocity PID request
  VoltageOut voltsOut = new VoltageOut(0.0); // for SysId

  SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new Config(
      Volts.of(0.6).per(Seconds.of(1.0)), // ramp us 0.6 V/s
      Volts.of(6), // mas voltage 6V
      Seconds.of(5), // run for at most 10 seconds
      (state) -> SignalLogger.writeString("state", state.toString())
    ),
    new Mechanism(volts -> intakeMotor.setControl(voltsOut.withOutput(volts.in(Volts))), null, this)
  );

  public Intake() {
    intakeMotor.getConfigurator().apply(VELOCITY_PID_CONFIGS);
    intakeMotor.getConfigurator().apply(MOTION_MAGIC_CONFIGS);
  }

  public Command runIntake(Measure<Velocity<Angle>> velTarget) {
    return runEnd(
        () -> intakeMotor.setControl(velocityRequest.withVelocity(velTarget.in(RotationsPerSecond))) ,
        () -> intakeMotor.setControl(velocityRequest.withVelocity(0.0))
      );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeVeltarget_rps", getTargetVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber("intakeVelCurrent_rps", getTargetVelocity().in(RotationsPerSecond));
  }

  public Command sysIDQuasistatic(Direction dir) {
    return sysIdRoutine.quasistatic(dir);
  }

  public Command sysIDDynamic(Direction dir) {
    return sysIdRoutine.dynamic(dir);
  }

  public Measure<Velocity<Angle>> getVelocity() {
    return RotationsPerSecond.of(intakeMotor.getVelocity().getValueAsDouble());
  }

  public Measure<Velocity<Angle>> getTargetVelocity() {
    return RotationsPerSecond.of(intakeMotor.getClosedLoopReference().getValueAsDouble());
  }
}
