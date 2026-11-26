package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/**
 * The hardware side of vision. Vision.java only talks to this interface, so the same logic runs
 * with the real Limelight (VisionIOLimelight) or the fake one in simulation (VisionIOSim).
 */
public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        /** The Limelight is powered, on the network, and sending new frames. */
        public boolean connected = false;
        /** The camera sees an AprilTag right now. */
        public boolean hasTarget = false;
        /** ID of the tag it's tracking, or -1 when it sees nothing. */
        public int tagId = -1;
        /** How far left/right the tag is from the center of the image. Right is positive (Limelight's tx). */
        public double txDegrees = 0.0;
        /** How far up/down the tag is from the center of the image. Up is positive (Limelight's ty). */
        public double tyDegrees = 0.0;
        /** How much of the image the tag fills, 0 to 100 (Limelight's ta). */
        public double areaPercent = 0.0;
        /**
         * Straight-line distance from the camera lens to the tag's center. The Limelight works this out
         * from how big the tag looks, so it doesn't need the camera's height or tilt.
         */
        public double distanceMeters = 0.0;
        /** How old the data is: time to process the frame plus time to capture it. */
        public double latencyMs = 0.0;
    }

    /** Read the newest camera data into inputs. Called once per loop. */
    default void updateInputs(VisionIOInputs inputs) {}
}
