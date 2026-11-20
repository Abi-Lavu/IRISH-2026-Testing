package frc.robot.subsystems.drive.requests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import frc.robot.Constants.DriveConstants;

/**
 * A request is one way of driving. The drivetrain is handed a request with {@code
 * drive.setControl(request)} and turns it into four wheel outputs. Same idea as CTRE's
 * SwerveRequest, adapted to a mecanum drive with no encoders (all speeds are fractions of full
 * power, -1 to 1).
 *
 * <p>Usage: build each request once, then update it with the "with" methods every loop.
 *
 * <pre>
 * private final MecanumRequest.FieldCentric fieldCentric = new MecanumRequest.FieldCentric();
 * drive.applyRequest(() -> fieldCentric.withVelocityX(fwd).withVelocityY(left).withRotationalRate(turn));
 * </pre>
 */
public interface MecanumRequest {

  /** What the drivetrain knows when it applies a request. */
  record DriveState(Rotation2d heading, boolean gyroConnected) {}

  /** What a request tells the drivetrain to do: wheel outputs and whether to brake at zero. */
  record DriveOutput(WheelSpeeds speeds, boolean brake) {
    static DriveOutput stopped(boolean brake) {
      return new DriveOutput(new WheelSpeeds(), brake);
    }
  }

  DriveOutput apply(DriveState state);

  // ---------------------------------------------------------------------------------------------
  // Stopping
  // ---------------------------------------------------------------------------------------------

  /** All wheels off, motors in coast. */
  final class Idle implements MecanumRequest {
    @Override
    public DriveOutput apply(DriveState state) {
      return DriveOutput.stopped(false);
    }
  }

  /** All wheels off, motors in brake. Mecanum can't "X-lock" like swerve, so this is the best stop. */
  final class Brake implements MecanumRequest {
    @Override
    public DriveOutput apply(DriveState state) {
      return DriveOutput.stopped(true);
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Driving
  // ---------------------------------------------------------------------------------------------

  /** Shared stick inputs and deadbands for the driving requests. */
  abstract class TranslationRequest<T extends TranslationRequest<T>> implements MecanumRequest {
    protected double velocityX; // forward +
    protected double velocityY; // left +
    protected double deadband;

    @SuppressWarnings("unchecked")
    private T self() {
      return (T) this;
    }

    /** Forward is positive, -1 to 1. */
    public T withVelocityX(double velocityX) {
      this.velocityX = velocityX;
      return self();
    }

    /** Left is positive, -1 to 1. */
    public T withVelocityY(double velocityY) {
      this.velocityY = velocityY;
      return self();
    }

    /** Stick values smaller than this count as zero. */
    public T withDeadband(double deadband) {
      this.deadband = deadband;
      return self();
    }

    protected double x() {
      return MathUtil.applyDeadband(velocityX, deadband);
    }

    protected double y() {
      return MathUtil.applyDeadband(velocityY, deadband);
    }
  }

  /** Robot-relative: "forward" is the way the robot's front points. Works with no gyro. */
  final class RobotCentric extends TranslationRequest<RobotCentric> {
    private double rotationalRate;
    private double rotationalDeadband;

    /** Counterclockwise is positive, -1 to 1. */
    public RobotCentric withRotationalRate(double rotationalRate) {
      this.rotationalRate = rotationalRate;
      return this;
    }

    public RobotCentric withRotationalDeadband(double rotationalDeadband) {
      this.rotationalDeadband = rotationalDeadband;
      return this;
    }

    @Override
    public DriveOutput apply(DriveState state) {
      double rot = MathUtil.applyDeadband(rotationalRate, rotationalDeadband);
      return new DriveOutput(MecanumDrive.driveCartesianIK(x(), y(), rot), true);
    }
  }

  /**
   * Field-relative: "forward" is away from the driver, whichever way the robot faces. If the gyro is
   * missing or unplugged, this quietly drives robot-relative instead so the robot stays driveable.
   */
  final class FieldCentric extends TranslationRequest<FieldCentric> {
    private double rotationalRate;
    private double rotationalDeadband;

    /** Counterclockwise is positive, -1 to 1. */
    public FieldCentric withRotationalRate(double rotationalRate) {
      this.rotationalRate = rotationalRate;
      return this;
    }

    public FieldCentric withRotationalDeadband(double rotationalDeadband) {
      this.rotationalDeadband = rotationalDeadband;
      return this;
    }

    @Override
    public DriveOutput apply(DriveState state) {
      double rot = MathUtil.applyDeadband(rotationalRate, rotationalDeadband);
      Rotation2d heading = state.gyroConnected() ? state.heading() : Rotation2d.kZero;
      return new DriveOutput(MecanumDrive.driveCartesianIK(x(), y(), rot, heading), true);
    }
  }

  /** Turns the robot to face a direction while it translates. Shared by the two facing-angle requests. */
  abstract class FacingAngleRequest<T extends FacingAngleRequest<T>> extends TranslationRequest<T> {
    protected Rotation2d targetDirection = Rotation2d.kZero;
    protected final PIDController headingController =
        new PIDController(DriveConstants.kHeadingP, 0.0, 0.0);

    protected FacingAngleRequest() {
      // 179 degrees and -179 degrees are only 2 degrees apart.
      headingController.enableContinuousInput(-180.0, 180.0);
    }

    @SuppressWarnings("unchecked")
    public T withTargetDirection(Rotation2d targetDirection) {
      this.targetDirection = targetDirection;
      return (T) this;
    }

    /** Turn output toward the target. Zero without a gyro (we can't know which way we face). */
    protected double rotation(DriveState state) {
      if (!state.gyroConnected()) {
        return 0.0;
      }
      double output =
          headingController.calculate(state.heading().getDegrees(), targetDirection.getDegrees());
      return MathUtil.clamp(
          output, -DriveConstants.kMaxHeadingCorrection, DriveConstants.kMaxHeadingCorrection);
    }
  }

  /** Field-relative driving while the gyro points the robot at a field direction. */
  final class FieldCentricFacingAngle extends FacingAngleRequest<FieldCentricFacingAngle> {
    @Override
    public DriveOutput apply(DriveState state) {
      Rotation2d heading = state.gyroConnected() ? state.heading() : Rotation2d.kZero;
      return new DriveOutput(
          MecanumDrive.driveCartesianIK(x(), y(), rotation(state), heading), true);
    }
  }

  /** Robot-relative driving while the gyro holds a heading. Keeps timed drives straight. */
  final class RobotCentricFacingAngle extends FacingAngleRequest<RobotCentricFacingAngle> {
    @Override
    public DriveOutput apply(DriveState state) {
      return new DriveOutput(MecanumDrive.driveCartesianIK(x(), y(), rotation(state)), true);
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Testing
  // ---------------------------------------------------------------------------------------------

  /** Set each wheel directly, -1 to 1. For checking motor directions on blocks. */
  final class WheelPercents implements MecanumRequest {
    private final WheelSpeeds speeds = new WheelSpeeds();

    public WheelPercents withWheels(
        double frontLeft, double frontRight, double rearLeft, double rearRight) {
      speeds.frontLeft = frontLeft;
      speeds.frontRight = frontRight;
      speeds.rearLeft = rearLeft;
      speeds.rearRight = rearRight;
      return this;
    }

    @Override
    public DriveOutput apply(DriveState state) {
      return new DriveOutput(speeds, true);
    }
  }
}
