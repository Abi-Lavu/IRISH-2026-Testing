package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.requests.MecanumRequest;
import frc.robot.subsystems.drive.requests.MecanumRequest.DriveOutput;
import frc.robot.subsystems.drive.requests.MecanumRequest.DriveState;
import java.util.function.Supplier;

/**
 * Mecanum drivetrain: 4 CIMs on Talon SRX, no drive encoders, optional gyro.
 *
 * <p>Everything drives through requests: {@code setControl(request)} or the {@code
 * applyRequest(...)} command. See {@link MecanumRequest} for the list.
 */
public class DriveSubsystem extends SubsystemBase {
  private final TalonSRX m_frontLeft = new TalonSRX(DriveConstants.kFrontLeftMotorId);
  private final TalonSRX m_rearLeft = new TalonSRX(DriveConstants.kRearLeftMotorId);
  private final TalonSRX m_frontRight = new TalonSRX(DriveConstants.kFrontRightMotorId);
  private final TalonSRX m_rearRight = new TalonSRX(DriveConstants.kRearRightMotorId);

  private final Gyro m_gyro = createGyro();

  /** Stops the wheels if no request arrives for 0.1 s (code froze, command ended, etc.). */
  private final MotorSafety m_safety =
      new MotorSafety() {
        @Override
        public void stopMotor() {
          setAllWheels(new WheelSpeeds());
        }

        @Override
        public String getDescription() {
          return "Mecanum Drive";
        }
      };

  private MecanumRequest m_request = new MecanumRequest.Idle();
  private boolean m_brake = true;
  private WheelSpeeds m_lastSpeeds = new WheelSpeeds();

  public DriveSubsystem() {
    configureMotor(m_frontLeft, DriveConstants.kFrontLeftInverted);
    configureMotor(m_rearLeft, DriveConstants.kRearLeftInverted);
    configureMotor(m_frontRight, DriveConstants.kFrontRightInverted);
    configureMotor(m_rearRight, DriveConstants.kRearRightInverted);

    m_safety.setExpiration(0.1);
    m_safety.setSafetyEnabled(true);

    // When nothing else is driving, sit still in brake.
    MecanumRequest brake = new MecanumRequest.Brake();
    setDefaultCommand(applyRequest(() -> brake).withName("Brake"));
  }

  /** The only place that knows which gyro we have. Never throws: a bad gyro means "no gyro". */
  private static Gyro createGyro() {
    switch (DriveConstants.kGyroType) {
      case PIGEON2:
        return new Pigeon2Gyro(DriveConstants.kPigeonId);
      case NAVX:
        // To use a navX: install the Studica vendordep, copy NavXGyro.java into this folder, and
        // replace these two lines with: return new NavXGyro();
        DriverStation.reportWarning("navX selected but NavXGyro is not installed, no gyro", false);
        return new NoGyro();
      case NONE:
      default:
        return new NoGyro();
    }
  }

  private static void configureMotor(TalonSRX motor, boolean inverted) {
    motor.configFactoryDefault();

    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.voltageCompSaturation = DriveConstants.kVoltageCompSaturation;
    config.continuousCurrentLimit = DriveConstants.kCurrentLimitAmps;
    config.peakCurrentLimit = 0;
    config.peakCurrentDuration = 0;
    config.openloopRamp = DriveConstants.kOpenLoopRampSeconds;
    motor.configAllSettings(config);

    motor.setInverted(inverted);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.enableVoltageCompensation(true);
    motor.enableCurrentLimit(true);
  }

  // ---------------------------------------------------------------------------------------------
  // Requests
  // ---------------------------------------------------------------------------------------------

  /** Drive with this request right now. Call it every loop (the default command does). */
  public void setControl(MecanumRequest request) {
    m_request = request;
    DriveOutput output = request.apply(getState());

    setBrake(output.brake());

    WheelSpeeds s = output.speeds();
    double max = DriveConstants.kMaxOutput;
    setAllWheels(
        new WheelSpeeds(s.frontLeft * max, s.frontRight * max, s.rearLeft * max, s.rearRight * max));
    m_safety.feed();
  }

  /** A command that keeps applying the request from {@code requestSupplier} every loop. */
  public Command applyRequest(Supplier<MecanumRequest> requestSupplier) {
    return run(() -> setControl(requestSupplier.get())).withName("Apply Request");
  }

  public DriveState getState() {
    return new DriveState(m_gyro.getHeading(), m_gyro.isConnected());
  }

  private void setAllWheels(WheelSpeeds speeds) {
    m_lastSpeeds = speeds;
    m_frontLeft.set(ControlMode.PercentOutput, speeds.frontLeft);
    m_rearLeft.set(ControlMode.PercentOutput, speeds.rearLeft);
    m_frontRight.set(ControlMode.PercentOutput, speeds.frontRight);
    m_rearRight.set(ControlMode.PercentOutput, speeds.rearRight);
  }

  /** Only sends the neutral mode over CAN when it actually changes. */
  private void setBrake(boolean brake) {
    if (brake == m_brake) {
      return;
    }
    m_brake = brake;
    NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    m_frontLeft.setNeutralMode(mode);
    m_rearLeft.setNeutralMode(mode);
    m_frontRight.setNeutralMode(mode);
    m_rearRight.setNeutralMode(mode);
  }

  // ---------------------------------------------------------------------------------------------
  // Heading
  // ---------------------------------------------------------------------------------------------

  public Rotation2d getHeading() {
    return m_gyro.getHeading();
  }

  public boolean isGyroConnected() {
    return m_gyro.isConnected();
  }

  /** Make the way the robot faces now "forward" for field-centric driving. */
  public void seedFieldCentric() {
    m_gyro.zero();
  }

  /** No requirement on purpose: re-zeroing must not interrupt driving. Works while disabled. */
  public Command seedFieldCentricCommand() {
    return Commands.runOnce(this::seedFieldCentric).ignoringDisable(true).withName("Seed Field Centric");
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Drive/Request", m_request.getClass().getSimpleName());
    SmartDashboard.putBoolean("Drive/Gyro Connected", m_gyro.isConnected());
    SmartDashboard.putNumber("Drive/Heading (deg)", getHeading().getDegrees());
    SmartDashboard.putBoolean("Drive/Brake", m_brake);
    SmartDashboard.putNumberArray(
        "Drive/Wheels [FL, FR, RL, RR]",
        new double[] {
          m_lastSpeeds.frontLeft, m_lastSpeeds.frontRight,
          m_lastSpeeds.rearLeft, m_lastSpeeds.rearRight
        });
  }
}
