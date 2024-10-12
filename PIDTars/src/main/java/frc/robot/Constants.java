// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

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

    public static final Slot0Configs VELOCITY_PID_CONFIGS = new Slot0Configs()
      .withKV(0.11162)
      .withKS(0.24348)
      .withKA(0.0084362)
      .withKG(0.019283)
      .withKP(0.16787)
      .withKD(0.0);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(9999)
      .withMotionMagicAcceleration(9999)
      .withMotionMagicJerk(9999);
  }
}
