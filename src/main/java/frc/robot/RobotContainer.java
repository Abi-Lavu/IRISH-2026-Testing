package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.requests.MecanumRequest;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.WantedState;
import frc.robot.subsystems.transfer.TransferSubsystem;

/**
 * Builds the robot: creates every subsystem and connects the driver's Xbox controller (port 0).
 * Nothing here reads the dashboard, so the robot drives from the Driver Station and controller
 * alone.
 */
public class RobotContainer {
  // ---- Subsystems ----
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final TransferSubsystem m_transfer = new TransferSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // Built last on purpose: it needs the other three.
  private final Superstructure m_superstructure =
      new Superstructure(m_intake, m_transfer, m_shooter);

  private final CommandXboxController m_driver =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // ---- Drive requests: built once, updated every loop ----
  private final MecanumRequest.FieldCentric m_fieldCentric =
      new MecanumRequest.FieldCentric()
          .withDeadband(DriveConstants.kDeadband)
          .withRotationalDeadband(DriveConstants.kDeadband);
  private final MecanumRequest.RobotCentric m_robotCentric =
      new MecanumRequest.RobotCentric()
          .withDeadband(DriveConstants.kDeadband)
          .withRotationalDeadband(DriveConstants.kDeadband);
  private final MecanumRequest.FieldCentricFacingAngle m_facingAngle =
      new MecanumRequest.FieldCentricFacingAngle().withDeadband(DriveConstants.kDeadband);
  private final MecanumRequest.Brake m_brake = new MecanumRequest.Brake();

  private boolean m_fieldCentricEnabled = DriveConstants.kFieldRelativeByDefault;
  private boolean m_slowMode = false;

  public RobotContainer() {
    configureDrive();
    configureSuperstructure();
  }

  // Xbox sticks: pushing forward gives negative Y, pushing right gives positive X. WPILib wants
  // forward, left and counterclockwise positive, so all three are flipped.
  private double forward() {
    return -m_driver.getLeftY() * speedScale();
  }

  private double left() {
    return -m_driver.getLeftX() * speedScale();
  }

  private double turn() {
    return -m_driver.getRightX() * speedScale();
  }

  private double speedScale() {
    return m_slowMode ? DriveConstants.kSlowModeScale : 1.0;
  }

  private void configureDrive() {
    m_drive.setDefaultCommand(
        m_drive
            .applyRequest(
                () ->
                    m_fieldCentricEnabled
                        ? m_fieldCentric
                            .withVelocityX(forward())
                            .withVelocityY(left())
                            .withRotationalRate(turn())
                        : m_robotCentric
                            .withVelocityX(forward())
                            .withVelocityY(left())
                            .withRotationalRate(turn()))
            .withName("Teleop Drive"));

    // Hold left bumper: slow mode. No requirement, so driving keeps going.
    m_driver
        .leftBumper()
        .whileTrue(Commands.startEnd(() -> m_slowMode = true, () -> m_slowMode = false));

    // Start: the way the robot faces now becomes "forward". Back: toggle field/robot-centric.
    m_driver.start().onTrue(m_drive.seedFieldCentricCommand());
    m_driver.back().onTrue(Commands.runOnce(() -> m_fieldCentricEnabled = !m_fieldCentricEnabled));

    // Hold X: stop and brake.
    m_driver.x().whileTrue(m_drive.applyRequest(() -> m_brake).withName("Brake"));

    // Hold the D-pad: snap to face that field direction while still driving with the left stick.
    // Needs a gyro; without one it just drives with no turning.
    snapWhileHeld(0, 0); // up: face away from driver
    snapWhileHeld(90, 90); // left
    snapWhileHeld(180, 180); // down: face the driver
    snapWhileHeld(270, -90); // right
  }

  private void snapWhileHeld(int povAngle, double headingDeg) {
    Rotation2d target = Rotation2d.fromDegrees(headingDeg);
    m_driver
        .pov(povAngle)
        .whileTrue(
            m_drive
                .applyRequest(
                    () ->
                        m_facingAngle
                            .withVelocityX(forward())
                            .withVelocityY(left())
                            .withTargetDirection(target))
                .withName("Snap " + headingDeg));
  }

  private void configureSuperstructure() {
    double t = OperatorConstants.kTriggerThreshold;
    // Hold a button = want that state, let go = back to IDLE.
    m_driver.rightTrigger(t).whileTrue(m_superstructure.holdStateCommand(WantedState.INTAKE));
    m_driver.leftTrigger(t).whileTrue(m_superstructure.holdStateCommand(WantedState.PREPARE_SHOT));
    m_driver.rightBumper().whileTrue(m_superstructure.holdStateCommand(WantedState.SHOOT));
    m_driver.b().whileTrue(m_superstructure.holdStateCommand(WantedState.EJECT));
  }

  /** No autos yet: the robot sits still during autonomous. */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
