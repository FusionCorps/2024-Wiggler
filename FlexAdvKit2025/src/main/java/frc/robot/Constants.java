// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Map;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class IntakeConstants {
    public static enum IntakeState {
      IDLE(0.0),
      INTAKE(0.8),
      EXTAKE(-0.8);

      public double pct = 0.0;

      private IntakeState(double pct) {
        this.pct = pct;
      }
    }

    public static final int INTAKE_MOTOR_ID = 6;
    public static final int BEAM_BREAK_SENSOR_ID = 1;
  }

  public class DriveConstants {
    public static enum SysIdRoutineMode {
      Drive,
      Steer,
      Rotation
    }
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains =
        new Slot0Configs()
            .withKP(100)
            .withKI(0)
            .withKD(0.0)
            .withKS(0.03428)
            .withKV(0.22857)
            .withKA(0.0014722)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains =
        new Slot0Configs()
            .withKP(0.4)
            .withKI(0)
            .withKD(0)
            .withKS(0.08223)
            .withKV(0.097754)
            .withKA(0.02484);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("*", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.573);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3;

    private static final double kDriveGearRatio = 5.142857142857142;
    private static final double kSteerGearRatio = 12.8;
    private static final Distance kWheelRadius = Inches.of(2.194);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 0;

    // These are only used for simulation
    private static final double kSteerInertia = 0.01;
    private static final double kDriveInertia = 0.025;
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.25);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(cancoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 13;
    private static final int kFrontLeftSteerMotorId = 14;
    private static final int kFrontLeftEncoderId = 15;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.2119140625);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftCANcoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(9.75);
    private static final Distance kFrontLeftYPos = Inches.of(9.75);

    // Front Right
    private static final int kFrontRightDriveMotorId = 19;
    private static final int kFrontRightSteerMotorId = 20;
    private static final int kFrontRightEncoderId = 21;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.26513671875);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightCANcoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(9.75);
    private static final Distance kFrontRightYPos = Inches.of(-9.75);

    // Back Left
    private static final int kBackLeftDriveMotorId = 16;
    private static final int kBackLeftSteerMotorId = 17;
    private static final int kBackLeftEncoderId = 18;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.35205078125);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftCANcoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-9.75);
    private static final Distance kBackLeftYPos = Inches.of(9.75);

    // Back Right
    private static final int kBackRightDriveMotorId = 10;
    private static final int kBackRightSteerMotorId = 11;
    private static final int kBackRightEncoderId = 12;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.32763671875);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightCANcoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-9.75);
    private static final Distance kBackRightYPos = Inches.of(-9.75);

    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId,
                kFrontLeftDriveMotorId,
                kFrontLeftEncoderId,
                kFrontLeftEncoderOffset,
                kFrontLeftXPos,
                kFrontLeftYPos,
                kInvertLeftSide,
                kFrontLeftSteerMotorInverted,
                kFrontLeftCANcoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId,
                kFrontRightDriveMotorId,
                kFrontRightEncoderId,
                kFrontRightEncoderOffset,
                kFrontRightXPos,
                kFrontRightYPos,
                kInvertRightSide,
                kFrontRightSteerMotorInverted,
                kFrontRightCANcoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId,
                kBackLeftDriveMotorId,
                kBackLeftEncoderId,
                kBackLeftEncoderOffset,
                kBackLeftXPos,
                kBackLeftYPos,
                kInvertLeftSide,
                kBackLeftSteerMotorInverted,
                kBackLeftCANcoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId,
                kBackRightDriveMotorId,
                kBackRightEncoderId,
                kBackRightEncoderOffset,
                kBackRightXPos,
                kBackRightYPos,
                kInvertRightSide,
                kBackRightSteerMotorInverted,
                kBackRightCANcoderInverted);

    // PathPlanner config constants
    public static final double WHEEL_COF = 1.19;
    public static final Mass ROBOT_MASS = Pounds.of(115.0);
    public static final double ROBOT_MOI = 2.0; // TODO: find more accurate value

    public static final RobotConfig PP_CONFIG =
        new RobotConfig(
            DriveConstants.ROBOT_MASS.in(Kilograms),
            DriveConstants.ROBOT_MOI,
            new ModuleConfig(
                FrontLeft.WheelRadius,
                kSpeedAt12Volts.in(MetersPerSecond),
                DriveConstants.WHEEL_COF,
                DCMotor.getKrakenX60Foc(1).withReduction(FrontLeft.DriveMotorGearRatio),
                FrontLeft.SlipCurrent,
                1),
            Drive.getModuleTranslations());

    public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(DriveConstants.ROBOT_MASS)
            .withCustomModuleTranslations(Drive.getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                () ->
                    new SwerveModuleSimulation(
                        new SwerveModuleSimulationConfig(
                            DCMotor.getKrakenX60(1),
                            DCMotor.getFalcon500(1),
                            FrontLeft.DriveMotorGearRatio,
                            FrontLeft.SteerMotorGearRatio,
                            Volts.of(FrontLeft.DriveFrictionVoltage),
                            Volts.of(FrontLeft.SteerFrictionVoltage),
                            Meters.of(FrontLeft.WheelRadius),
                            KilogramSquareMeters.of(FrontLeft.SteerInertia),
                            DriveConstants.WHEEL_COF)));
  }

  public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout APRILTAG_LAYOUT;

    // Camera names, must match names configured on coprocessor
    public static String CAM_0_NAME = "camera_0";
    public static String CAM_1_NAME = "camera_1";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0)); // facing forward
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI)); // facing backward

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.08; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static class ShooterConstants {
    public static enum ShooterState {
      IDLE(RPM.of(0), RPM.of(0)),
      SPEAKER(RPM.of(3000), RPM.of(5000)),
      AMP(RPM.of(-1200), RPM.of(-1200)),
      SHUTTLE(RPM.of(2900), RPM.of(2900)),
      EXTAKE(RPM.of(1500), RPM.of(1500));

      public AngularVelocity topRPM;
      public AngularVelocity bottomRPM;

      private ShooterState(AngularVelocity topRPM, AngularVelocity bottomRPM) {
        this.topRPM = topRPM;
        this.bottomRPM = bottomRPM;
      }
    }

    public static final int SHOOTER_TOP_MOTOR_ID = 2;
    public static final int SHOOTER_BOTTOM_MOTOR_ID = 3;

    public static boolean IS_SHOOTING_RIGHT = false;

    public static boolean HAS_STOPPED_REVVING = false;
    public static boolean IS_AMP = false;

    public static final LinearVelocity ShooterSpeed =
        FeetPerSecond.of(28.06308713961776); // in ft/s

    public static final AngularVelocity SHOOTER_MAX_RPM = RPM.of(6784);

    // TODO: tune further: get both faster
    public static final double SHOOTER_kP = 0.0004;
    public static final double SHOOTER_kI = 0.0;
    public static final double SHOOTER_kD = 0.006;
    public static final double SHOOTER_kFF = 0.000176;

    public static final Current SHOOTER_STALL_LIMIT_CURRENT = Amps.of(60); // in amps
    public static final Current SHOOTER_FREE_LIMIT_CURRENT = Amps.of(30); // in amps
    public static final AngularVelocity SHOOTER_FREE_SPEED_LIMIT = RPM.of(5500); // in RPM
  }

  public static class PivotConstants {
    public static enum PivotState {
      INTAKE(Rotations.of(22.5)),
      INTAKE_SIM(Rotations.of(-0.1)),
      SUBWOOFER(Rotations.of(5.0947265625)),
      SUBWOOFER_SIM(Rotations.of(-0.166)),
      AMP(Rotations.of(99)),
      AMP_SIM(Rotations.of(0.31)),
      SHUTTLE(Rotations.of(14.853271484375)),
      SHUTTLE_SIM(Rotations.of(-0.02)),
      EXTAKE(Rotations.of(22.5)),
      EXTAKE_SIM(Rotations.of(-0.1)),
      MANUAL_OVERRIDE(Rotations.of(0)),
      AIMING(Rotations.of(0));

      public Angle angle;

      private PivotState(Angle pivotAngle) {
        this.angle = pivotAngle;
      }
    }

    public static final double PIVOT_GEAR_RATIO = 31.25;

    public static final double PIVOT_ERROR_THRESHOLD = 0.2;

    public static final int PIVOT_MAIN_MOTOR_ID = 1;
    public static final int PIVOT_FOLLOWER_MOTOR_ID = 5;

    public static final double PIVOT_kV = 0.0;
    public static final double PIVOT_kP = 8.0;
    public static final double PIVOT_kI = 0.0;
    public static final double PIVOT_kD = 0.01;
    public static final double PIVOT_kG = 0.001;

    // motion magic constraints
    public static final double PIVOT_CRUISE_VELOCITY = 2500;
    public static final double PIVOT_ACCELERATION = 2000;
    public static final double PIVOT_JERK = 50000;

    // maps Z distances to april tag (meters) with pivot angles (rotations)
    // these won't work now, since Flex's Limelight was recalibrated
    public static final InterpolatingDoubleTreeMap PIVOT_ANGLES_MAP_REAL =
        InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.1, PivotState.INTAKE.angle.in(Rotations)),
            Map.entry(1.43, 21.68115234375),
            Map.entry(1.84, 16.1193359375),
            Map.entry(2.31, 12.78369140625),
            Map.entry(2.76, 10.56689453125),
            Map.entry(2.82, 10.32265625),
            Map.entry(3.02, 9.54326171875),
            Map.entry(3.46, 8.09541015625));

    public static final InterpolatingDoubleTreeMap
        PIVOT_ANGLES_MAP_SIM = // maps distance from speaker to angle for pivot (in rotations)
        InterpolatingDoubleTreeMap.ofEntries(
                Map.entry(1.176, PivotState.INTAKE_SIM.angle.in(Rotations)),
                Map.entry(1.5, -0.138044),
                Map.entry(2.0, -0.113583),
                Map.entry(2.533, -0.092616),
                Map.entry(3.0, -0.082133),
                Map.entry(3.5, -0.075144),
                Map.entry(4.0, -0.07165),
                Map.entry(4.5, -0.064661),
                Map.entry(5.384, -0.057670));
  }

  public static class IndexConstants {
    public static final int INDEX_MOTOR_ID = 4;
    public static final double INDEX_RUN_PCT = .28;
    public static final double INDEX_AMP_PCT = .30;

    public static enum IndexState {
      IDLE(0.0),
      SPEAKER(0.28),
      INTAKE(0.28),
      EXTAKE(0.28),
      AMP(0.30);

      public double pct = 0.0;

      private IndexState(double pct) {
        this.pct = pct;
      }
    }
  }
}
