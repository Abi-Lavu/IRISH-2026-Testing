package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

/**
 * Mecanum drivetrain, state-based like Irish27.
 *
 * The sticks are handed in once. Every loop, periodic() looks at the current State and decides
 * the wheel outputs. Commands only "hold" a state while they run, then it goes back to MANUAL.
 *
 * The Talons are in brake mode (see DriveIOTalonSRX), so letting go of the sticks stops the robot.
 */
public class Drive extends SubsystemBase {
    public enum State {
        MANUAL, // full-speed driving from the sticks
        SLOW // driving from the sticks, scaled down
    }

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final DoubleSupplier forward, strafe, turn;
    private State state = State.MANUAL;

    /** Stops the wheels if periodic() stops running for 0.1 s (the code froze). */
    private final MotorSafety safety = new MotorSafety() {
        @Override
        public void stopMotor() {
            io.setOutputs(0, 0, 0, 0);
        }

        @Override
        public String getDescription() {
            return "Mecanum Drive";
        }
    };

    /**
     * @param forward forward is positive, -1 to 1
     * @param strafe left is positive, -1 to 1
     * @param turn counterclockwise is positive, -1 to 1
     */
    public Drive(DriveIO io, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.io = io;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;

        safety.setExpiration(0.1);
        safety.setSafetyEnabled(true);
    }

    // ---- Commands: hold a state while scheduled, back to MANUAL when they end ----

    /** Slow driving for lining up (and later, while intaking). */
    public Command slowMode() {
        return hold(State.SLOW).withName("SlowMode");
    }

    private Command hold(State newState) {
        return startEnd(() -> state = newState, () -> state = State.MANUAL);
    }

    public State getState() {
        return state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        double x = 0, y = 0, z = 0;
        // Nobody may drive during autonomous, so the sticks are ignored there.
        boolean sticksAllowed = DriverStation.isTeleopEnabled();
        switch (state) {
            case MANUAL -> {
                if (sticksAllowed) {
                    x = forward.getAsDouble();
                    y = strafe.getAsDouble();
                    z = turn.getAsDouble();
                }
            }
            case SLOW -> {
                if (sticksAllowed) {
                    x = forward.getAsDouble() * DriveConstants.SlowModeScale;
                    y = strafe.getAsDouble() * DriveConstants.SlowModeScale;
                    z = turn.getAsDouble() * DriveConstants.SlowModeScale;
                }
            }
        }

        // WPILib's mecanum math: (forward, left, turn) -> 4 wheel outputs, scaled to fit -1..1.
        WheelSpeeds wheels = MecanumDrive.driveCartesianIK(x, y, z);
        double max = DriveConstants.MaxOutput;
        io.setOutputs(
            wheels.frontLeft * max, wheels.frontRight * max, wheels.rearLeft * max, wheels.rearRight * max);
        safety.feed();

        Logger.recordOutput("Drive/State", state);
        Logger.recordOutput("Drive/Command", new double[] {x, y, z});
        Logger.recordOutput("Drive/WheelSetpoints",
            new double[] {wheels.frontLeft * max, wheels.frontRight * max, wheels.rearLeft * max, wheels.rearRight * max});
        logHealth();
    }

    /** One-glance health: are all four Talons talking, and is any of them too hot? */
    private void logHealth() {
        String[] names = {"FL", "FR", "BL", "BR"};
        boolean allConnected = true;
        boolean anyTooHot = false;
        StringBuilder problems = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            if (!inputs.connected[i]) {
                allConnected = false;
                problems.append(names[i]).append(" disconnected  ");
            }
            if (inputs.tempC[i] > DriveConstants.TooHotTempC) {
                anyTooHot = true;
                problems.append(names[i]).append(String.format(" hot (%.0f C)  ", inputs.tempC[i]));
            }
        }
        Logger.recordOutput("Drive/Health/AllConnected", allConnected);
        Logger.recordOutput("Drive/Health/AnyTooHot", anyTooHot);
        Logger.recordOutput("Drive/Health/Healthy", allConnected && !anyTooHot);
        Logger.recordOutput("Drive/Health/Problems", problems.length() == 0 ? "None" : problems.toString().trim());
    }
}
