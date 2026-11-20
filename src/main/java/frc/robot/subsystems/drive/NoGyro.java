package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/** Stand-in when the robot has no gyro. Field-centric requests fall back to robot-centric. */
public class NoGyro implements Gyro {
  @Override
  public Rotation2d getHeading() {
    return Rotation2d.kZero;
  }

  @Override
  public void zero() {}

  @Override
  public boolean isConnected() {
    return false;
  }
}
