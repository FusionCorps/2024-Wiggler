// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

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

    public static final Translation2d frontLeftLocation =
        new Translation2d(TRACK_WIDTH_METERS / 2.0, TRACK_LENGTH_METERS / 2.0);
    public static final Translation2d frontRightLocation =
        new Translation2d(TRACK_WIDTH_METERS / 2.0, -TRACK_LENGTH_METERS / 2.0);
    public static final Translation2d backLeftLocation =
        new Translation2d(-TRACK_WIDTH_METERS / 2.0, TRACK_LENGTH_METERS / 2.0);
    public static final Translation2d backRightLocation =
        new Translation2d(-TRACK_WIDTH_METERS / 2.0, -TRACK_LENGTH_METERS / 2.0);

    // MOTOR IDS for the SWERVE
    public static int TURN_FL_ID = 0;
    public static int TURN_FR_ID = 3;
    public static int TURN_BL_ID = 1;
    public static int TURN_BR_ID = 7;

    public static int DRIVE_FL_ID = 8;
    public static int DRIVE_FR_ID = 4;
    public static int DRIVE_BL_ID = 9;
    public static int DRIVE_BR_ID = 5;

    // CANCODER IDS
    public static int CODER_FL_ID = 12;
    public static int CODER_FR_ID = 13;
    public static int CODER_BL_ID = 2;
    public static int CODER_BR_ID = 11;

    public static final Slot0Configs TURN_PID_CONFIGS =
        new Slot0Configs().withKP(1.0).withKI(0.0).withKD(0.0).withKV(0.0).withKA(0.0).withKS(0.0);

    public static double STEERING_RATIO = 12.8;
    public static double DRIVING_RATIO = 6.75;

    public static double WHEEL_RADIUS_METERS = 0.0508;

    public static final Slot0Configs DRIVE_PID_CONFIGS =
        new Slot0Configs().withKP(1.0).withKI(0.0).withKD(0.0).withKV(0.0).withKA(0.0).withKS(0.0);
  }
}
