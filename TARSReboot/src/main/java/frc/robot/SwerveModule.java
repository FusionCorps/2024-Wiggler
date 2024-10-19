package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private TalonFX driveMotor, turnMotor;
  private CANcoder encoder;

  // voltage requests for steer/drive
  MotionMagicVoltage turnSetter = new MotionMagicVoltage(0.0);
  VoltageOut driveSetter = new VoltageOut(0.0);

  public SwerveModule(
      int driveMotorID, int turnMotorID, int encoderID, double encoderOffset, boolean invert) {
    driveMotor = new TalonFX(driveMotorID);
    turnMotor = new TalonFX(turnMotorID);
    encoder = new CANcoder(encoderID);

    TalonFXConfiguration driveConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        invert
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(DRIVE_PID_CONFIGS)
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(120)
                    .withStatorCurrentLimitEnable(true));

    driveMotor.getConfigurator().apply(driveConfig);

    TalonFXConfiguration turnConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(TURN_PID_CONFIGS)
            .withCurrentLimits(new CurrentLimitsConfigs())
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withFeedbackRemoteSensorID(encoderID)
                    .withRotorToSensorRatio(STEERING_RATIO))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0 / STEERING_RATIO)
                    .withMotionMagicAcceleration(100.0 / STEERING_RATIO / 0.100)
                    .withMotionMagicExpo_kV(0.12 * STEERING_RATIO)
                    .withMotionMagicExpo_kA(0.1));

    ClosedLoopGeneralConfigs closedLoopGeneral = new ClosedLoopGeneralConfigs();
    closedLoopGeneral.ContinuousWrap = true;
    turnConfig.ClosedLoopGeneral = closedLoopGeneral;

    turnMotor.getConfigurator().apply(turnConfig);

    encoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(encoderOffset)));
  }

  public void setDesiredState(SwerveModuleState state) {
    var optimizedState =
        SwerveModuleState.optimize(
            state, Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble()));

    double velocityToSet = optimizedState.speedMetersPerSecond;
    double angleToSet = optimizedState.angle.getRotations();

    driveMotor.setControl(driveSetter.withOutput(velocityToSet / kMaxSpeed * 12.0));
    turnMotor.setControl(turnSetter.withPosition(angleToSet));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition().getValueAsDouble()
            * DRIVING_RATIO
            * (2 * Math.PI * WHEEL_RADIUS_METERS),
        Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble()));
  }
}
