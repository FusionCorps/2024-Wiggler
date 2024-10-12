package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.ControlMode;
import frc.robot.TunableNumber;
import static edu.wpi.first.units.Units.Volts;

public class Arm extends SubsystemBase {

  // hardware/simulation motor models
  TalonFX armMotor = new TalonFX(0);
  TalonFXSimState armMotorSim = new TalonFXSimState(armMotor);
  DCMotorSim armModel = new DCMotorSim(DCMotor.getKrakenX60(1), 1.0, 0.01);

  // dashboard Mechanisms for visualization 
  final Mechanism2d angleMech = new Mechanism2d(1, 1);
  final MechanismRoot2d angleRoot = angleMech.getRoot("center", 0.5, 0.5);
  final MechanismLigament2d armAngleMech = angleRoot.append(new MechanismLigament2d("arm", 0.3, 0.0));
  final MechanismLigament2d armTargetAngleMech = angleRoot.append(
    new MechanismLigament2d("armTarget", 0.3, 0.0, 6.0, new Color8Bit(0, 255, 0)));
  
  final Mechanism2d velMech = new Mechanism2d(1, 1);
  final MechanismRoot2d velRoot = velMech.getRoot("center", 0.5, 0.5);
  final MechanismLigament2d armVelMech = velRoot.append(new MechanismLigament2d("arm", 0, 180));
  final MechanismLigament2d armTargetVelMech = velRoot.append(
    new MechanismLigament2d("armTarget", 0, 180, 6.0, new Color8Bit(0, 255, 0)));

  // control mode
  ControlMode controlMode = ControlMode.Position;

  // control signals
  MotionMagicVoltage posReq = new MotionMagicVoltage(0.0); // kV=0.112, kP=0.9, kI=0.0, kD=0.0, kS=0.0
  MotionMagicVelocityVoltage velReq = new MotionMagicVelocityVoltage(0.0); // kV=0.112, kP=0.9, kI=0.0, kD=0.0, kS=0.0
  VoltageOut sysIdVoltageRequest = new VoltageOut(0.0);

  // sysid routine
  SysIdRoutine armSysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,
      Volts.of(6),
      null,
      state -> SignalLogger.writeString("state", state.toString())
    ),
    new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> armMotor.setControl(sysIdVoltageRequest.withOutput(volts.in(Volts))), null, this));

  // tunable numbers on dashboard
  TunableNumber kP = new TunableNumber("kP");
  TunableNumber kI = new TunableNumber("kI");
  TunableNumber kD = new TunableNumber("kD");

  Slot0Configs posConfigs = new Slot0Configs();
  Slot1Configs velConfigs = new Slot1Configs();

  public Arm() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    posConfigs.kP = Constants.defkPPos;
    posConfigs.kI = Constants.defkIPos;
    posConfigs.kD = Constants.defkDPos;
    posConfigs.kG = Constants.defkG;

    velConfigs.kP = Constants.defkPVel;
    velConfigs.kI = Constants.defkIVel;
    velConfigs.kD = Constants.defkDVel;
    velConfigs.kV = Constants.defkV;
    velConfigs.kS = Constants.defkS;
    velConfigs.kG = Constants.defkG;

    config.Slot0 = posConfigs;
    config.Slot1 = velConfigs;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = 9999;
    config.MotionMagic.MotionMagicAcceleration = 9999;
    config.MotionMagic.MotionMagicJerk = 9999;
    
    armMotor.getConfigurator().apply(config);

    armMotor.setPosition(0.0);

    // post the mechanism to the dashboard
    SmartDashboard.putData("Arm Angle", angleMech);
    SmartDashboard.putData("Arm Velocity", velMech);

    // post the tunable numbers to the dashboard
    if (controlMode == ControlMode.Position) {
      kP.setDefault(Constants.defkPPos);
      kI.setDefault(Constants.defkIPos);
      kD.setDefault(Constants.defkDPos);
    } else {
      kP.setDefault(Constants.defkPVel);
      kI.setDefault(Constants.defkIVel);
      kD.setDefault(Constants.defkDVel);
    }
    
    BaseStatusSignal.setUpdateFrequencyForAll(250,
        armMotor.getPosition(),
        armMotor.getVelocity(),
        armMotor.getMotorVoltage());
    
    armMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    // dynamic PID tuning
    if (controlMode == ControlMode.Position) {
      if (posConfigs.kP != kP.get() || posConfigs.kI != kI.get() || posConfigs.kD != kD.get()) {
        posConfigs.kP = kP.get();
        posConfigs.kI = kI.get();
        posConfigs.kD = kD.get();
        armMotor.getConfigurator().apply(posConfigs);
      }
    } else {
      if (velConfigs.kP != kP.get() || velConfigs.kI != kI.get() || velConfigs.kD != kD.get()) {
        velConfigs.kP = kP.get();
        velConfigs.kI = kI.get();
        velConfigs.kD = kD.get();
        armMotor.getConfigurator().apply(velConfigs);
      }
    }

    // telemetry
    armAngleMech.setAngle(Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble()));
    armVelMech.setLength(armMotor.getVelocity().getValueAsDouble() * 60.0 / Constants.FALCON_500_FREE_SPEED * 0.3);
    SmartDashboard.putNumber("armAngleDeg", Units.rotationsToDegrees(armMotor.getPosition().getValue()));
    SmartDashboard.putNumber("armVelRPM", armMotor.getVelocity().getValue() * 60.0);
    SmartDashboard.putString("Operation Mode", controlMode.toString());
  }

  @Override
  public void simulationPeriodic() {
    // feed motor inputs from CTRE sim to physics model, then outputs into CTRE motor
    armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    armModel.setInputVoltage(armMotorSim.getMotorVoltage());
    SmartDashboard.putNumber("armMotorVoltage", armMotorSim.getMotorVoltage());
    armModel.update(0.020);

    armMotorSim.setRawRotorPosition(armModel.getAngularPositionRotations());
    armMotorSim.setRotorVelocity(Units.radiansToRotations(armModel.getAngularVelocityRadPerSec()));
  }

  public Command setTargetPosition(DoubleSupplier pos) {
    return runOnce(() -> controlMode = ControlMode.Position).andThen(
      run(() -> {
      armMotor.setControl(posReq.withPosition(pos.getAsDouble()).withSlot(Constants.POSITION_GAIN_SLOT));

      armTargetAngleMech.setAngle(Units.rotationsToDegrees(posReq.Position));
      SmartDashboard.putNumber("targetAngleDeg", Units.rotationsToDegrees(posReq.Position));
    }));
  }

  public Command setTargetVelocity(DoubleSupplier vel) {
    return runOnce(() -> controlMode = ControlMode.Velocity).andThen(
    run(() -> {
      armMotor.setControl(velReq.withVelocity(vel.getAsDouble()).withSlot(Constants.VELOCITY_GAIN_SLOT));
      
      armTargetVelMech.setLength(velReq.Velocity * 60.0 / Constants.FALCON_500_FREE_SPEED * 0.3);
      SmartDashboard.putNumber("targetVelocityRPM", velReq.Velocity * 60.0);
    }));
  }

  public Command sysIDQuasistatic(Direction dir) {
    return armSysIdRoutine.quasistatic(dir);
  }

  public Command sysIDDynamic(Direction dir) {
    return armSysIdRoutine.dynamic(dir);
  }
}
