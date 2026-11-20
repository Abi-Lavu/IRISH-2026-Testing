package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Anything that can tell us which way the robot is facing.
 *
 * <p>The drive code only talks to this interface, never to a navX or Pigeon directly. That way,
 * switching gyros is one line in Constants and nothing else changes.
 */
public interface Gyro {
  /** Robot heading. Counterclockwise is positive, which is what WPILib expects. */
  Rotation2d getHeading();

  /** Make the direction the robot is facing right now count as 0 degrees ("forward"). */
  void zero();

  /** False if the gyro is unplugged or not talking. Shown on the dashboard. */
  boolean isConnected();
}
