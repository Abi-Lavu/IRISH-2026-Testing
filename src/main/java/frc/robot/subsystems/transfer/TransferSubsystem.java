package frc.robot.subsystems.transfer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

/**
 * Hopper roller floor: one NEO drives all 7 floor rollers plus the side wheels that push pieces
 * into the shooter's feeder. The Superstructure picks the goal.
 */
public class TransferSubsystem extends SubsystemBase {

  public enum Goal {
    OFF(0.0),
    INTAKE(TransferConstants.kIntakeVolts), // gentle: pull new pieces into the hopper
    FEED(TransferConstants.kFeedVolts), // fast: push pieces into the shooter
    REVERSE(TransferConstants.kReverseVolts); // unjam / eject

    public final double volts;

    Goal(double volts) {
      this.volts = volts;
    }
  }

  private final SparkMax m_motor = new SparkMax(TransferConstants.kMotorId, MotorType.kBrushless);
  private Goal m_goal = Goal.OFF;

  public TransferSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TransferConstants.kCurrentLimit)
        .inverted(TransferConstants.kInverted);
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setGoal(Goal goal) {
    m_goal = goal;
  }

  @Override
  public void periodic() {
    m_motor.setVoltage(m_goal.volts);
    SmartDashboard.putString("Transfer/Goal", m_goal.name());
    SmartDashboard.putNumber("Transfer/Current (A)", m_motor.getOutputCurrent());
  }
}
