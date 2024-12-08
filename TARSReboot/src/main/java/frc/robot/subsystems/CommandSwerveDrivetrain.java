package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DrivetrainConstants.kSpeedAt12VoltsMps;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveSteerGains;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveTranslation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  /**
   * Enum to specify the mode of the SysId routine. Translation: System identification for
   * translation. SteerGains: System identification for steer gains Rotation: System identification
   * for rotation (angular motion)
   */
  public static enum SysIdMode {
    Translation,
    SteerGains,
    Rotation
  }

  private final SysIdMode sysIdMode = SysIdMode.Translation; // MODIFY THIS TO CHANGE SYSID MODE

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // Field centric request
  private FieldCentric fieldCentricRequest =
      new FieldCentric()
          .withSteerRequestType(SteerRequestType.MotionMagic)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDeadband(kSpeedAt12VoltsMps * 0.1)
          .withRotationalDeadband(Math.PI * 0.1);

  // Brake & point requests
  private SwerveDriveBrake brakeRequest = new SwerveDriveBrake();
  private PointWheelsAt pointWheelsAtRequest = new PointWheelsAt();

  // Sysid requests
  private SysIdSwerveTranslation sysIdSwerveTranslation = new SysIdSwerveTranslation();
  private SysIdSwerveSteerGains sysIdSwerveSteerGains = new SysIdSwerveSteerGains();
  private SysIdSwerveRotation sysIdSwerveRotation = new SysIdSwerveRotation();

  private SysIdRoutine sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Seconds.of(1)),
              Volts.of(2),
              Seconds.of(5),
              state -> SignalLogger.writeString("stateTranslation", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(sysIdSwerveTranslation.withVolts(volts)), null, this));

  private SysIdRoutine sysIdRoutineSteerGains =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> SignalLogger.writeString("stateSteerGains", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(sysIdSwerveSteerGains.withVolts(volts)), null, this));

  private SysIdRoutine sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> SignalLogger.writeString("stateRotationGains", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(sysIdSwerveRotation.withVolts(volts)), null, this));

  /** Alliance logic * */
  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);

  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  // Handles actual requests. Abstracted through other methods.
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing
     */
    // if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    //   DriverStation.getAlliance()
    //       .ifPresent(
    //           (allianceColor) -> {
    //             this.setOperatorPerspectiveForward(
    //                 allianceColor == Alliance.Red
    //                     ? RedAlliancePerspectiveRotation
    //                     : BlueAlliancePerspectiveRotation);
    //             hasAppliedOperatorPerspective = true;
    //           });
    // }
  }

  /**
   * Run the swerve drivetrain in field-centric mode.
   *
   * @param forward Supplier for forward movement.
   * @param strafe Supplier for strafing movement.
   * @param rotate Supplier for rotational movement.
   * @return Command
   */
  public Command runSwerveFC(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate) {
    return applyRequest(
        () ->
            fieldCentricRequest
                .withVelocityX(forward.getAsDouble())
                .withVelocityY(strafe.getAsDouble())
                .withRotationalRate(rotate.getAsDouble()));
  }

  // Brake command
  public Command brake() {
    return applyRequest(() -> brakeRequest);
  }

  // Point command
  public Command pointWheelsAt(DoubleSupplier x, DoubleSupplier y) {
    return applyRequest(
        () ->
            pointWheelsAtRequest.withModuleDirection(
                new Rotation2d(x.getAsDouble(), y.getAsDouble())));
  }

  /**
   * Command to run SysId, based on enum defined above.
   *
   * @param isDynamic true for dynamic, false for quasistatic.
   * @param direction Direction to run the SysId in, forward or reverse.
   */
  public Command runSysId(boolean isDynamic, Direction direction) {
    switch (sysIdMode) {
      case Translation:
        return isDynamic
            ? sysIdRoutineTranslation.dynamic(direction)
            : sysIdRoutineTranslation.quasistatic(direction);
      case SteerGains:
        return isDynamic
            ? sysIdRoutineSteerGains.dynamic(direction)
            : sysIdRoutineSteerGains.quasistatic(direction);
      case Rotation:
        return isDynamic
            ? sysIdRoutineRotation.dynamic(direction)
            : sysIdRoutineRotation.quasistatic(direction);
      default:
        return Commands.none();
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
