package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Over-the-bumper intake: one NEO spins the two tube rollers, one NEO swings the intake out and in.
 *
 * <p>This subsystem never decides anything by itself. The Superstructure tells it a goal for the
 * pivot and a goal for the rollers, and periodic() makes the motors do it.
 */
public class IntakeSubsystem extends SubsystemBase {

  /** Where the intake should be. Each goal carries its own angle. */
  public enum PivotGoal {
    STOWED(IntakeConstants.kStowedDeg),
    DEPLOYED(IntakeConstants.kDeployedDeg);

    public final double angleDeg;

    PivotGoal(double angleDeg) {
      this.angleDeg = angleDeg;
    }
  }

  /** What the rollers should do. Each goal carries its own voltage. */
  public enum RollerGoal {
    OFF(0.0),
    INTAKE(IntakeConstants.kRollerIntakeVolts),
    EJECT(IntakeConstants.kRollerEjectVolts);

    public final double volts;

    RollerGoal(double volts) {
      this.volts = volts;
    }
  }

  private final SparkMax m_roller = new SparkMax(IntakeConstants.kRollerMotorId, MotorType.kBrushless);
  private final SparkMax m_pivot = new SparkMax(IntakeConstants.kPivotMotorId, MotorType.kBrushless);
  private final RelativeEncoder m_pivotEncoder = m_pivot.getEncoder();

  // A profiled PID plans a smooth trapezoid path to the goal angle, then follows it.
  private final ProfiledPIDController m_pivotController =
      new ProfiledPIDController(
          IntakeConstants.kPivotP,
          0.0,
          IntakeConstants.kPivotD,
          new TrapezoidProfile.Constraints(
              IntakeConstants.kPivotMaxVelocityDegPerSec,
              IntakeConstants.kPivotMaxAccelDegPerSec2));

  private PivotGoal m_pivotGoal = PivotGoal.STOWED;
  private RollerGoal m_rollerGoal = RollerGoal.OFF;

  public IntakeSubsystem() {
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.kRollerCurrentLimit)
        .inverted(IntakeConstants.kRollerInverted);
    m_roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
        .idleMode(IdleMode.kBrake) // brake so it doesn't flop down when disabled
        .smartCurrentLimit(IntakeConstants.kPivotCurrentLimit)
        .inverted(IntakeConstants.kPivotInverted);
    // Make the encoder read pivot DEGREES instead of motor rotations.
    // One motor turn = 360 / gear ratio degrees of pivot.
    pivotConfig.encoder.positionConversionFactor(360.0 / IntakeConstants.kPivotGearRatio);
    pivotConfig.encoder.velocityConversionFactor(360.0 / IntakeConstants.kPivotGearRatio / 60.0);
    m_pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // The NEO's built-in encoder only counts from where it started. RULE: the intake must be
    // STOWED when the robot turns on, so "power on" = "stowed angle".
    m_pivotEncoder.setPosition(IntakeConstants.kStowedDeg);
    m_pivotController.reset(IntakeConstants.kStowedDeg);
  }

  // ---- Called by the Superstructure ----

  public void setPivotGoal(PivotGoal goal) {
    m_pivotGoal = goal;
  }

  public void setRollerGoal(RollerGoal goal) {
    m_rollerGoal = goal;
  }

  public double getPivotAngleDeg() {
    return m_pivotEncoder.getPosition();
  }

  /** True when the pivot is close enough to its goal angle. */
  public boolean isPivotAtGoal() {
    return Math.abs(getPivotAngleDeg() - m_pivotGoal.angleDeg) < IntakeConstants.kPivotToleranceDeg;
  }

  @Override
  public void periodic() {
    double angleDeg = getPivotAngleDeg();

    if (DriverStation.isDisabled()) {
      // While disabled, keep the profile starting from wherever the intake really is, so it
      // doesn't jump when we enable.
      m_pivotController.reset(angleDeg);
    } else {
      double pidVolts = m_pivotController.calculate(angleDeg, m_pivotGoal.angleDeg);
      // Gravity help: 0 when upright (0 deg), strongest when level (90 deg). Gravity pulls the
      // intake OUT (toward bigger angles), so we push back with a negative voltage.
      double gravityVolts =
          -IntakeConstants.kPivotGravityVolts * Math.sin(Math.toRadians(angleDeg));
      m_pivot.setVoltage(pidVolts + gravityVolts);
    }

    m_roller.setVoltage(m_rollerGoal.volts);

    SmartDashboard.putString("Intake/Pivot Goal", m_pivotGoal.name());
    SmartDashboard.putString("Intake/Roller Goal", m_rollerGoal.name());
    SmartDashboard.putNumber("Intake/Pivot Angle (deg)", angleDeg);
    SmartDashboard.putBoolean("Intake/Pivot At Goal", isPivotAtGoal());
  }
}
