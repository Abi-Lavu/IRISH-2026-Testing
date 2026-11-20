package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Single-flywheel shooter with a vertical feeder. Two NEOs:
 *
 * <ul>
 *   <li>Flywheel: closed-loop speed control (feedforward + P) so every shot leaves at the same
 *       speed.
 *   <li>Feeder: simple voltage, lifts one piece at a time into the flywheel.
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {

  public enum FlywheelGoal {
    OFF,
    SPIN_UP // spin to the target RPM (set with setTargetRpm)
  }

  public enum FeederGoal {
    OFF(0.0),
    FEED(ShooterConstants.kFeederFeedVolts),
    REVERSE(ShooterConstants.kFeederReverseVolts);

    public final double volts;

    FeederGoal(double volts) {
      this.volts = volts;
    }
  }

  private final SparkMax m_flywheel =
      new SparkMax(ShooterConstants.kFlywheelMotorId, MotorType.kBrushless);
  private final SparkMax m_feeder =
      new SparkMax(ShooterConstants.kFeederMotorId, MotorType.kBrushless);
  private final RelativeEncoder m_flywheelEncoder = m_flywheel.getEncoder();

  private final PIDController m_flywheelPid = new PIDController(ShooterConstants.kFlywheelP, 0, 0);

  private FlywheelGoal m_flywheelGoal = FlywheelGoal.OFF;
  private FeederGoal m_feederGoal = FeederGoal.OFF;
  private double m_targetRpm = ShooterConstants.kShotRpm;

  public ShooterSubsystem() {
    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig
        .idleMode(IdleMode.kCoast) // coast: let the heavy flywheel spin down on its own
        .smartCurrentLimit(ShooterConstants.kFlywheelCurrentLimit)
        .inverted(ShooterConstants.kFlywheelInverted);
    // Make the encoder read FLYWHEEL RPM instead of motor RPM.
    flywheelConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kFlywheelGearRatio);
    // The NEO encoder averages speed over a window by default, which makes the reading lag.
    // A shorter window = a fresher speed reading = faster "at speed" detection.
    flywheelConfig.encoder.uvwMeasurementPeriod(16).uvwAverageDepth(2);
    m_flywheel.configure(
        flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig
        .idleMode(IdleMode.kBrake) // brake: hold pieces still when not feeding
        .smartCurrentLimit(ShooterConstants.kFeederCurrentLimit)
        .inverted(ShooterConstants.kFeederInverted);
    m_feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ---- Called by the Superstructure ----

  public void setFlywheelGoal(FlywheelGoal goal) {
    m_flywheelGoal = goal;
  }

  public void setFeederGoal(FeederGoal goal) {
    m_feederGoal = goal;
  }

  public void setTargetRpm(double rpm) {
    m_targetRpm = rpm;
  }

  public double getFlywheelRpm() {
    return m_flywheelEncoder.getVelocity();
  }

  /** True when the flywheel is spinning and close enough to the target to shoot. */
  public boolean isAtSpeed() {
    return m_flywheelGoal == FlywheelGoal.SPIN_UP
        && Math.abs(getFlywheelRpm() - m_targetRpm) < ShooterConstants.kRpmTolerance;
  }

  @Override
  public void periodic() {
    if (m_flywheelGoal == FlywheelGoal.SPIN_UP) {
      // Feedforward: the voltage we EXPECT to need for this speed (does most of the work).
      double ffVolts =
          ShooterConstants.kFlywheelS * Math.signum(m_targetRpm)
              + ShooterConstants.kFlywheelV * m_targetRpm;
      // Feedback: a small correction for whatever the feedforward got wrong.
      double pidVolts = m_flywheelPid.calculate(getFlywheelRpm(), m_targetRpm);
      // Never drive the flywheel backward to slow it down: just let it coast.
      m_flywheel.setVoltage(MathUtil.clamp(ffVolts + pidVolts, 0.0, 12.0));
    } else {
      m_flywheel.setVoltage(0.0);
    }

    m_feeder.setVoltage(m_feederGoal.volts);

    SmartDashboard.putString("Shooter/Flywheel Goal", m_flywheelGoal.name());
    SmartDashboard.putString("Shooter/Feeder Goal", m_feederGoal.name());
    SmartDashboard.putNumber("Shooter/Target RPM", m_targetRpm);
    SmartDashboard.putNumber("Shooter/Flywheel RPM", getFlywheelRpm());
    SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed());
  }
}
