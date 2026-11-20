package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.PivotGoal;
import frc.robot.subsystems.intake.IntakeSubsystem.RollerGoal;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.FeederGoal;
import frc.robot.subsystems.shooter.ShooterSubsystem.FlywheelGoal;
import frc.robot.subsystems.transfer.TransferSubsystem;

/**
 * The one place that decides what every game-piece mechanism does. (The drivetrain stays separate:
 * the driver always controls it directly.)
 *
 * <p>How it works, in two layers:
 *
 * <ol>
 *   <li><b>WantedState</b> is what the driver asked for ("I want to shoot").
 *   <li><b>SystemState</b> is what the robot is actually doing right now. It can differ from the
 *       wanted state. If the driver wants to SHOOT but the flywheel is still slow, the system
 *       state is SPINNING_UP, and the feeder stays off until the flywheel is ready.
 * </ol>
 *
 * <p>Every 20 ms, periodic() does three things: pick the next SystemState from the WantedState and
 * the sensors, tell each subsystem its goal for that state, and log everything.
 *
 * <p>Rule: buttons only ever call setWantedState(). Nothing else sets subsystem goals.
 */
public class Superstructure extends SubsystemBase {

  /** What the driver asked for. */
  public enum WantedState {
    IDLE, // everything off, intake tucked in
    INTAKE, // intake out, pull pieces into the hopper
    EJECT, // run everything backward to spit pieces out the front
    PREPARE_SHOT, // spin the flywheel up but don't shoot yet
    SHOOT // shoot as soon as the flywheel is ready
  }

  /** What the robot is actually doing. */
  public enum SystemState {
    IDLE,
    INTAKING,
    EJECTING,
    SPINNING_UP, // flywheel not at speed yet, feeder off
    READY, // flywheel at speed, waiting for the shoot button
    SHOOTING // flywheel at speed, feeder and transfer pushing pieces in
  }

  private final IntakeSubsystem m_intake;
  private final TransferSubsystem m_transfer;
  private final ShooterSubsystem m_shooter;

  private WantedState m_wantedState = WantedState.IDLE;
  private SystemState m_systemState = SystemState.IDLE;

  public Superstructure(
      IntakeSubsystem intake, TransferSubsystem transfer, ShooterSubsystem shooter) {
    m_intake = intake;
    m_transfer = transfer;
    m_shooter = shooter;
  }

  public void setWantedState(WantedState state) {
    m_wantedState = state;
  }

  public WantedState getWantedState() {
    return m_wantedState;
  }

  public SystemState getSystemState() {
    return m_systemState;
  }

  @Override
  public void periodic() {
    SystemState next = handleTransitions();
    if (next != m_systemState) {
      SmartDashboard.putString("Superstructure/Last Transition", m_systemState + " -> " + next);
    }
    m_systemState = next;

    applyState(m_systemState);

    SmartDashboard.putString("Superstructure/Wanted State", m_wantedState.name());
    SmartDashboard.putString("Superstructure/System State", m_systemState.name());
  }

  /** Step 1: decide which SystemState we should be in. All the "if" logic lives here. */
  private SystemState handleTransitions() {
    switch (m_wantedState) {
      case INTAKE:
        return SystemState.INTAKING;

      case EJECT:
        return SystemState.EJECTING;

      case PREPARE_SHOT:
        return m_shooter.isAtSpeed() ? SystemState.READY : SystemState.SPINNING_UP;

      case SHOOT:
        // Once we start shooting, keep shooting until the button is released. Each piece makes
        // the flywheel dip below "at speed" for a moment; if we stopped feeding on every dip,
        // shots would stutter. The flywheel's PID brings the speed back while we keep feeding.
        if (m_systemState == SystemState.SHOOTING) {
          return SystemState.SHOOTING;
        }
        return m_shooter.isAtSpeed() ? SystemState.SHOOTING : SystemState.SPINNING_UP;

      case IDLE:
      default:
        return SystemState.IDLE;
    }
  }

  /** Step 2: tell every subsystem its goal for this state. No "if" logic here, just a table. */
  private void applyState(SystemState state) {
    switch (state) {
      case INTAKING:
        m_intake.setPivotGoal(PivotGoal.DEPLOYED);
        m_intake.setRollerGoal(RollerGoal.INTAKE);
        m_transfer.setGoal(TransferSubsystem.Goal.INTAKE);
        m_shooter.setFlywheelGoal(FlywheelGoal.OFF);
        m_shooter.setFeederGoal(FeederGoal.OFF); // feeder stopped = pieces wait in the hopper
        break;

      case EJECTING:
        m_intake.setPivotGoal(PivotGoal.DEPLOYED);
        m_intake.setRollerGoal(RollerGoal.EJECT);
        m_transfer.setGoal(TransferSubsystem.Goal.REVERSE);
        m_shooter.setFlywheelGoal(FlywheelGoal.OFF);
        m_shooter.setFeederGoal(FeederGoal.REVERSE);
        break;

      case SPINNING_UP:
      case READY:
        m_intake.setPivotGoal(PivotGoal.STOWED);
        m_intake.setRollerGoal(RollerGoal.OFF);
        m_transfer.setGoal(TransferSubsystem.Goal.OFF);
        m_shooter.setFlywheelGoal(FlywheelGoal.SPIN_UP);
        m_shooter.setFeederGoal(FeederGoal.OFF);
        break;

      case SHOOTING:
        m_intake.setPivotGoal(PivotGoal.STOWED);
        m_intake.setRollerGoal(RollerGoal.OFF);
        m_transfer.setGoal(TransferSubsystem.Goal.FEED);
        m_shooter.setFlywheelGoal(FlywheelGoal.SPIN_UP);
        m_shooter.setFeederGoal(FeederGoal.FEED);
        break;

      case IDLE:
      default:
        m_intake.setPivotGoal(PivotGoal.STOWED);
        m_intake.setRollerGoal(RollerGoal.OFF);
        m_transfer.setGoal(TransferSubsystem.Goal.OFF);
        m_shooter.setFlywheelGoal(FlywheelGoal.OFF);
        m_shooter.setFeederGoal(FeederGoal.OFF);
        break;
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Commands for buttons and autos
  // ---------------------------------------------------------------------------------------------

  /** Set the wanted state once and finish right away. Good for autos. */
  public Command setWantedStateCommand(WantedState state) {
    return runOnce(() -> setWantedState(state)).withName("Want " + state);
  }

  /**
   * Want {@code state} while the command runs, then go back to IDLE. Bind with whileTrue so
   * "hold the button" means "keep doing it" and letting go stops everything.
   */
  public Command holdStateCommand(WantedState state) {
    return startEnd(() -> setWantedState(state), () -> setWantedState(WantedState.IDLE))
        .withName("Hold " + state);
  }
}
