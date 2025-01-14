// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    public static final int kIntakeMotorPort = 25;
    public static final Measure<Velocity<Angle>> kIntakeSpeed = Rotations.of(3000).per(Minute);

    public static final Slot0Configs VELOCITY_PID_CONFIGS =
        new Slot0Configs()
            .withKV(0.11162)
            .withKS(0.24348)
            .withKA(0.0084362)
            .withKG(0.019283)
            .withKP(0.16787)
            .withKD(0.0);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(9999)
            .withMotionMagicAcceleration(9999)
            .withMotionMagicJerk(9999);
  }

  public static class DrivetrainConstants {
    public static final double TRACK_WIDTH_METERS = 0.4953;
    public static final double TRACK_LENGTH_METERS = 0.4953;
    public static final double kMaxSpeed = 6.0; // m/s
    public static final double kMaxAngularSpeed = Math.PI; // rad/s

    // Generated by the Tuner X Swerve Project Generator
    // https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains =
        new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains =
        new Slot0Configs()
            .withKP(0.1661)
            .withKI(0)
            .withKD(0)
            .withKS(0.11102)
            .withKV(0.11088)
            .withKA(0.010752);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 150.0;

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(60)
                    .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 4.73;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 12.8;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "";
    private static final int kPigeonId = 0;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANbusName(kCANbusName)
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory ConstantCreator =
        new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withCANcoderInitialConfigs(cancoderInitialConfigs);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 8;
    private static final int kFrontLeftSteerMotorId = 0;
    private static final int kFrontLeftEncoderId = 12;
    private static final double kFrontLeftEncoderOffset = -0.232421875;
    private static final boolean kFrontLeftSteerInvert = false;

    private static final double kFrontLeftXPosInches = 10;
    private static final double kFrontLeftYPosInches = 10;

    // Front Right
    private static final int kFrontRightDriveMotorId = 4;
    private static final int kFrontRightSteerMotorId = 3;
    private static final int kFrontRightEncoderId = 13;
    private static final double kFrontRightEncoderOffset = -0.0361328125;
    private static final boolean kFrontRightSteerInvert = false;

    private static final double kFrontRightXPosInches = 10;
    private static final double kFrontRightYPosInches = -10;

    // Back Left
    private static final int kBackLeftDriveMotorId = 9;
    private static final int kBackLeftSteerMotorId = 1;
    private static final int kBackLeftEncoderId = 2;
    private static final double kBackLeftEncoderOffset = 0.133056640625;
    private static final boolean kBackLeftSteerInvert = false;

    private static final double kBackLeftXPosInches = -10;
    private static final double kBackLeftYPosInches = 10;

    // Back Right
    private static final int kBackRightDriveMotorId = 5;
    private static final int kBackRightSteerMotorId = 7;
    private static final int kBackRightEncoderId = 11;
    private static final double kBackRightEncoderOffset = 0.158447265625;
    private static final boolean kBackRightSteerInvert = false;

    private static final double kBackRightXPosInches = -10;
    private static final double kBackRightYPosInches = -10;

    private static final SwerveModuleConstants FrontLeft =
        ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId,
                kFrontLeftDriveMotorId,
                kFrontLeftEncoderId,
                kFrontLeftEncoderOffset,
                Units.inchesToMeters(kFrontLeftXPosInches),
                Units.inchesToMeters(kFrontLeftYPosInches),
                kInvertLeftSide)
            .withSteerMotorInverted(kFrontLeftSteerInvert);
    private static final SwerveModuleConstants FrontRight =
        ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId,
                kFrontRightDriveMotorId,
                kFrontRightEncoderId,
                kFrontRightEncoderOffset,
                Units.inchesToMeters(kFrontRightXPosInches),
                Units.inchesToMeters(kFrontRightYPosInches),
                kInvertRightSide)
            .withSteerMotorInverted(kFrontRightSteerInvert);
    private static final SwerveModuleConstants BackLeft =
        ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId,
                kBackLeftDriveMotorId,
                kBackLeftEncoderId,
                kBackLeftEncoderOffset,
                Units.inchesToMeters(kBackLeftXPosInches),
                Units.inchesToMeters(kBackLeftYPosInches),
                kInvertLeftSide)
            .withSteerMotorInverted(kBackLeftSteerInvert);
    private static final SwerveModuleConstants BackRight =
        ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId,
                kBackRightDriveMotorId,
                kBackRightEncoderId,
                kBackRightEncoderOffset,
                Units.inchesToMeters(kBackRightXPosInches),
                Units.inchesToMeters(kBackRightYPosInches),
                kInvertRightSide)
            .withSteerMotorInverted(kBackRightSteerInvert);

    public static final CommandSwerveDrivetrain DriveTrain =
        new CommandSwerveDrivetrain(
            DrivetrainConstants, 250, FrontLeft, FrontRight, BackLeft, BackRight);
  }
}
