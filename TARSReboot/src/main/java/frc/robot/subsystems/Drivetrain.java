// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import java.util.function.DoubleSupplier;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  SwerveModule frontLeftModule = new SwerveModule(DRIVE_FL_ID, TURN_FL_ID, CODER_FL_ID, 0.0, true);
  SwerveModule frontRightModule =
      new SwerveModule(DRIVE_FR_ID, TURN_FR_ID, CODER_FR_ID, 0.0, false);
  SwerveModule backLeftModule = new SwerveModule(DRIVE_BL_ID, TURN_BL_ID, CODER_BL_ID, 0.0, true);
  SwerveModule backRightModule = new SwerveModule(DRIVE_BR_ID, TURN_BR_ID, CODER_BR_ID, 0.0, false);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
          },
          new Pose2d());

  public Drivetrain() {
    gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  private void updateOdometry() {
    m_odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          backLeftModule.getPosition(),
          backRightModule.getPosition()
        });
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public Command runSwerveFC(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot) {
    return run(
        () -> {
          drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), true, 0.02);
        });
  }
}
