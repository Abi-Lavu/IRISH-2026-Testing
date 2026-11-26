package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.VisionConstants;

/**
 * The real Limelight 2. It sends everything over NetworkTables in a table named after the camera
 * ("limelight" by default), so no vendor library is needed.
 *
 * <p>The pipeline must be set to AprilTags (family 36h11, tag size 165.1 mm) in the Limelight web
 * page, otherwise it won't report any tags or distances.
 */
public class VisionIOLimelight implements VisionIO {
    private final DoubleSubscriber heartbeat, tv, tid, tx, ty, ta, tl, cl;
    /** Where the tag is in 3D from the camera: [x right, y down, z forward, roll, pitch, yaw] in meters/degrees. */
    private final DoubleArraySubscriber targetPoseCameraSpace;
    /** Tells the Limelight to only look for hub tags. Kept as a field so it stays published. */
    private final DoubleArrayPublisher tagFilter;

    private double lastHeartbeat = Double.NaN;
    private double lastHeartbeatChangeTime = -1000.0;

    public VisionIOLimelight(String cameraName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(cameraName);
        // The heartbeat goes up by one every frame. If it stops changing, the camera is gone.
        heartbeat = table.getDoubleTopic("hb").subscribe(Double.NaN);
        tv = table.getDoubleTopic("tv").subscribe(0.0);
        tid = table.getDoubleTopic("tid").subscribe(-1.0);
        tx = table.getDoubleTopic("tx").subscribe(0.0);
        ty = table.getDoubleTopic("ty").subscribe(0.0);
        ta = table.getDoubleTopic("ta").subscribe(0.0);
        tl = table.getDoubleTopic("tl").subscribe(0.0);
        cl = table.getDoubleTopic("cl").subscribe(0.0);
        targetPoseCameraSpace = table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[0]);

        // Only track the hub tags, so a tag elsewhere never gets picked as "the" target.
        double[] hubIds = new double[VisionConstants.HubTagIds.length];
        for (int i = 0; i < hubIds.length; i++) {
            hubIds[i] = VisionConstants.HubTagIds[i];
        }
        tagFilter = table.getDoubleArrayTopic("fiducial_id_filters_set").publish();
        tagFilter.set(hubIds);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double beat = heartbeat.get();
        if (!Double.isNaN(beat) && beat != lastHeartbeat) {
            lastHeartbeat = beat;
            lastHeartbeatChangeTime = now;
        }
        inputs.connected = now - lastHeartbeatChangeTime < VisionConstants.DisconnectedAfterSeconds;

        boolean seesTag = inputs.connected && tv.get() == 1.0;
        inputs.hasTarget = seesTag;
        inputs.tagId = seesTag ? (int) tid.get() : -1;
        inputs.txDegrees = seesTag ? tx.get() : 0.0;
        inputs.tyDegrees = seesTag ? ty.get() : 0.0;
        inputs.areaPercent = seesTag ? ta.get() : 0.0;

        double[] pose = targetPoseCameraSpace.get();
        inputs.distanceMeters = seesTag && pose.length >= 3
            ? Math.sqrt(pose[0] * pose[0] + pose[1] * pose[1] + pose[2] * pose[2])
            : 0.0;

        inputs.latencyMs = inputs.connected ? tl.get() + cl.get() : 0.0;
    }
}
