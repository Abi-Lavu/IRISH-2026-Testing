package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * The hardware side of the drivetrain. Drive.java only talks to this interface, so the
 * logic never touches a Talon directly. Every array is ordered FL, FR, BL, BR.
 */
public interface DriveIO {
    @AutoLog
    class DriveIOInputs {
        public boolean[] connected = new boolean[4];
        public double[] appliedPercent = new double[4];
        public double[] statorAmps = new double[4];
        public double[] supplyAmps = new double[4];
        public double[] tempC = new double[4];
        public double[] busVolts = new double[4];
        public int[] faults = new int[4];
        public int[] stickyFaults = new int[4];
        /** Simulation only: where the fake robot is on the field. Stays at zero on the real robot. */
        public Pose2d simPose = Pose2d.kZero;
    }

    /** Read every motor's health into inputs. Called once per loop. */
    default void updateInputs(DriveIOInputs inputs) {}

    /** Percent output for each wheel, -1 to 1. */
    default void setOutputs(double frontLeft, double frontRight, double backLeft, double backRight) {}
}
