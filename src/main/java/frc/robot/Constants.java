// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

  public static class DriveConstants{
    // public static final int frontLeftMotorID = 22;
     // public static final int frontRightMotorID = 10;
    // public static final int backLeftMotorID = 8;
    // public static final int backRightMotorID = 1;
    public static final int frontLeftMotorID = 22;
    public static final int frontRightMotorID = 10;
    public static final int backLeftMotorID = 8;
    public static final int backRightMotorID = 1;

    public static final boolean frontLeftInverted = false;
    public static final boolean backLeftInverted = false;
    public static final boolean backRightInverted = true;
    public static final boolean frontRightInverted = true;

    public static final double Deadband = 0.1;
    public static final double MaxOutput = 0.8;
    public static final double VoltageCompSaturation = 11.0;
    public static final int CurrentLimitAmps = 40;
    public static final double OpenLoopRampSeconds = 0.2;

    /** Speed multiplier while the slow-mode button (left bumper) is held. */
    public static final double SlowModeScale = 0.4;

    /** A Talon hotter than this shows up in the drive health log. */
    public static final double TooHotTempC = 60.0;

    /** Simulation only (DriveIOSim). Wheel spacing measured from the CAD (Top Level Assembly). */
    public static class DriveSimConstants {
      /** Front-to-back distance between wheel centers. */
      public static final double SimWheelbaseMeters = 0.54;
      /** Left-to-right distance between wheel centers. */
      public static final double SimTrackWidthMeters = 0.68;
      /** Wheel surface speed at 100% output. CIM 5330 rpm / 10.71 ToughBox Micro x 6 in wheel = about 4 m/s free, minus losses. */
      public static final double SimMaxWheelSpeedMetersPerSec = 3.5;
      /** Where the fake robot starts on the field (meters from the blue corner, facing red). */
      public static final Pose2d SimStartPose = new Pose2d(2.0, 4.0, Rotation2d.kZero);
      /** Robot weight with battery and bumpers, for the physics sim (pushing balls, hitting things). */
      public static final double SimRobotMassKg = 52.0;
      /** Outside size of the bumpers (front-to-back, side-to-side), from the robot CAD. */
      public static final double SimBumperLengthMeters = 0.98;
      public static final double SimBumperWidthMeters = 0.95;
      /** How grippy the wheels are (mecanum on carpet is roughly 0.7). Limits how hard the robot can push. */
      public static final double SimWheelGrip = 0.7;
    }
  }
}
