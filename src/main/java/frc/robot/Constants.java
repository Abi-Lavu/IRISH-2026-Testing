// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int frontLeftMotorID = 
    public static final int frontRightMotorID = 
    public static final int backLeftMotorID = 
    public static final int backRightMotorID = 

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
  }
}
