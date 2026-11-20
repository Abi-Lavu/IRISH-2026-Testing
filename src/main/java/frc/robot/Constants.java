package frc.robot;

/**
 * Every number the robot code depends on lives here, grouped by mechanism.
 *
 * <p>Anything marked TODO is a placeholder: check it on the real robot before trusting it. See the
 * "IRISH 2026 robot mechanisms" doc for where each number comes from.
 */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    /** How far a trigger must be pulled (0 to 1) to count as "pressed". */
    public static final double kTriggerThreshold = 0.5;
  }

  public static final class DriveConstants {
    // ---- CAN IDs (TODO: placeholders from IrishGreen, confirm in Phoenix Tuner X) ----
    public static final int kFrontLeftMotorId = 5;
    public static final int kRearLeftMotorId = 4;
    public static final int kFrontRightMotorId = 6;
    public static final int kRearRightMotorId = 7;

    // ---- Inversions (TODO: confirm with the robot on blocks) ----
    public static final boolean kFrontLeftInverted = false;
    public static final boolean kRearLeftInverted = false;
    public static final boolean kFrontRightInverted = true;
    public static final boolean kRearRightInverted = true;

    // ---- Talon SRX settings (same as the Tier 1 code) ----
    public static final double kDeadband = 0.10; // stick values under 10% count as zero
    public static final double kMaxOutput = 0.8; // top speed = 80%
    public static final double kVoltageCompSaturation = 11.0; // "100%" means 11 V
    public static final int kCurrentLimitAmps = 40; // per-motor current cap
    public static final double kOpenLoopRampSeconds = 0.2; // 0 to full power in 0.2 s

    /** Speed multiplier while the slow-mode button is held. */
    public static final double kSlowModeScale = 0.4;

    // ---- Gyro ----
    /** Which gyro is on the robot. Only one of these is true at a time. */
    public enum GyroType {
      NONE, // no gyro: field-centric falls back to robot-centric
      PIGEON2,
      NAVX
    }

    public static final GyroType kGyroType = GyroType.NONE; // TODO: set once the gyro is mounted
    public static final int kPigeonId = 20; // TODO: confirm in Phoenix Tuner X (Pigeon only)

    /** Drive field-centric when the robot turns on (needs a gyro). The driver toggles it with Back. */
    public static final boolean kFieldRelativeByDefault = true;

    // ---- Heading control for the facing-angle requests (needs a gyro) ----
    /** Turn output per degree of heading error. 0.02 means 10 degrees off = 20% turn. */
    public static final double kHeadingP = 0.02;
    /** Never turn faster than this while correcting heading. */
    public static final double kMaxHeadingCorrection = 0.3;
  }

  public static final class IntakeConstants {
    // TODO: all CAN IDs are placeholders, set them in REV Hardware Client
    public static final int kRollerMotorId = 10;
    public static final int kPivotMotorId = 11;

    public static final boolean kRollerInverted = false;
    public static final boolean kPivotInverted = false;

    public static final int kRollerCurrentLimit = 40;
    public static final int kPivotCurrentLimit = 30;

    // Roller voltages (out of about 12 V). Positive pulls pieces in.
    public static final double kRollerIntakeVolts = 8.0;
    public static final double kRollerEjectVolts = -6.0;

    /**
     * Motor turns per pivot turn. The CAD shows a 12T to 36T belt (3:1). TODO: confirm with
     * mechanical, 3:1 is a very small reduction for a pivot.
     */
    public static final double kPivotGearRatio = 3.0;

    // Pivot angles in degrees. 0 = stowed upright inside the frame, bigger = further out.
    public static final double kStowedDeg = 0.0;
    public static final double kDeployedDeg = 110.0; // TODO: measure on the robot
    public static final double kPivotToleranceDeg = 5.0;

    // Pivot motion: a profile makes it move smoothly instead of slamming.
    public static final double kPivotMaxVelocityDegPerSec = 360.0;
    public static final double kPivotMaxAccelDegPerSec2 = 720.0;

    // Pivot PID (output is volts). TODO: tune. Start small and raise kP until it tracks.
    public static final double kPivotP = 0.10; // volts per degree of error
    public static final double kPivotD = 0.0;
    /** Volts needed to hold the intake level against gravity. TODO: tune. */
    public static final double kPivotGravityVolts = 0.5;
  }

  public static final class TransferConstants {
    public static final int kMotorId = 12; // TODO: placeholder
    public static final boolean kInverted = false;
    public static final int kCurrentLimit = 30;

    // Positive moves pieces toward the shooter.
    public static final double kIntakeVolts = 5.0; // gentle, while picking up
    public static final double kFeedVolts = 9.0; // fast, while shooting
    public static final double kReverseVolts = -6.0; // unjam / eject
  }

  public static final class ShooterConstants {
    public static final int kFeederMotorId = 13; // TODO: placeholder
    public static final int kFlywheelMotorId = 14; // TODO: placeholder

    public static final boolean kFeederInverted = false;
    public static final boolean kFlywheelInverted = false;

    public static final int kFeederCurrentLimit = 30;
    public static final int kFlywheelCurrentLimit = 60;

    // Feeder voltages. Positive moves a piece up into the flywheel.
    public static final double kFeederFeedVolts = 10.0;
    public static final double kFeederReverseVolts = -6.0;

    /** Motor turns per flywheel turn. The CAD shows 26T to 36T, so 36/26 = 1.385. */
    public static final double kFlywheelGearRatio = 36.0 / 26.0;

    /** Flywheel speed for a normal shot, in flywheel RPM. TODO: tune on the field. */
    public static final double kShotRpm = 3000.0;
    /** "At speed" means within this many RPM of the target. */
    public static final double kRpmTolerance = 75.0;

    // Flywheel feedforward + PID (output is volts). TODO: tune.
    // kV: a NEO spins ~5676 RPM at 12 V, so the flywheel tops out near 5676 / 1.385 = 4100 RPM.
    // So 12 V / 4100 RPM = about 0.0029 volts per RPM.
    public static final double kFlywheelS = 0.10; // volts to overcome friction
    public static final double kFlywheelV = 0.0029; // volts per flywheel RPM
    public static final double kFlywheelP = 0.0008; // extra volts per RPM of error
  }
}
