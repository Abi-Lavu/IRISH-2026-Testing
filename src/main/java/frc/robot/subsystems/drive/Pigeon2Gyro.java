package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/** A CTRE Pigeon 2.0 on the CAN bus (Phoenix 6 library, already installed). */
public class Pigeon2Gyro implements Gyro {
  private final Pigeon2 m_pigeon;
  private final StatusSignal<Angle> m_yaw;

  public Pigeon2Gyro(int canId) {
    m_pigeon = new Pigeon2(canId);
    m_yaw = m_pigeon.getYaw();
    m_pigeon.setYaw(0.0);
  }

  @Override
  public Rotation2d getHeading() {
    // Pigeon 2 yaw is already counterclockwise-positive.
    return m_pigeon.getRotation2d();
  }

  @Override
  public void zero() {
    m_pigeon.setYaw(0.0);
  }

  @Override
  public boolean isConnected() {
    return m_yaw.refresh().getStatus().isOK();
  }
}
