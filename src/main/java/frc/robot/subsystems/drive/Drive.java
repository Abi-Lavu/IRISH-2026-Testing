package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.RobotBase;
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
        // Debug: what the sticks read and whether the code is allowed to use them.
        Logger.recordOutput("Drive/Debug/SticksAllowed", sticksAllowed);
        Logger.recordOutput("Drive/Debug/RawSticks",
            new double[] {forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble()});
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

        // Mecanum math: (forward, left, counterclockwise) -> 4 wheel outputs, scaled to fit -1..1.
        WheelSpeeds wheels = mecanumIK(x, y, z);
        double max = DriveConstants.MaxOutput;
        io.setOutputs(
            wheels.frontLeft * max, wheels.frontRight * max, wheels.rearLeft * max, wheels.rearRight * max);
        safety.feed();

        Logger.recordOutput("Drive/State", state);
        Logger.recordOutput("Drive/Command", new double[] {x, y, z});
        Logger.recordOutput("Drive/WheelSetpoints",
            new double[] {wheels.frontLeft * max, wheels.frontRight * max, wheels.rearLeft * max, wheels.rearRight * max});
        logHealth();

        // No encoders or gyro on the real robot, so only the simulation knows where the robot is.
        if (RobotBase.isSimulation()) {
            Logger.recordOutput("Drive/Pose", inputs.simPose);
        }
    }

    /**
     * Turns (forward, left, counterclockwise) into wheel outputs for a standard X-roller mecanum.
     *
     * <p>We don't use WPILib's MecanumDrive.driveCartesianIK here: its signs treat "left" and
     * "counterclockwise" backwards from its own docs and from MecanumDriveKinematics, which made the
     * robot strafe and turn the wrong way. To strafe left, the front-left and back-right wheels spin
     * backward while the other two spin forward.
     */
    static WheelSpeeds mecanumIK(double forward, double left, double ccw) {
        double fl = forward - left - ccw;
        double fr = forward + left + ccw;
        double bl = forward + left - ccw;
        double br = forward - left + ccw;

        // If any wheel is asked for more than 100%, scale all four down together.
        double biggest = Math.max(1.0,
            Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        return new WheelSpeeds(fl / biggest, fr / biggest, bl / biggest, br / biggest);
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
