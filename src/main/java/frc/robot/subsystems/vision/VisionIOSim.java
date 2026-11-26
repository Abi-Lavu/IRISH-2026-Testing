package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.VisionSimConstants;
import frc.robot.PracticeFieldArena;

/**
 * Fake Limelight for simulation. It knows where the hub tags are on our practice field and where
 * the simulated robot is, and reports the closest tag the camera could see, the way the real
 * Limelight would.
 *
 * <p>The tags are REBUILT's hub tags (9, 10, 25, 26, manual section 3.10), moved from REBUILT's two
 * hubs onto our one hub in the middle of the field. 9 and 10 face the red wall, 25 and 26 face blue.
 */
public class VisionIOSim implements VisionIO {
    // Centers of the two REBUILT hubs, from the official tag layout.
    private static final Translation2d REBUILT_RED_HUB = new Translation2d(11.9155, 4.035);
    private static final Translation2d REBUILT_BLUE_HUB = new Translation2d(4.6255, 4.035);
    private static final Translation2d PRACTICE_HUB =
        new Translation2d(PracticeFieldArena.FIELD_LENGTH / 2, PracticeFieldArena.FIELD_WIDTH / 2);

    /** Side of the black square of a 2026 AprilTag. */
    private static final double TAG_SIZE_METERS = 0.1651;

    private final Supplier<Pose2d> robotPose;
    private final int[] ids = VisionConstants.HubTagIds;
    private final Pose3d[] tagPoses = new Pose3d[ids.length];

    /** @param robotPose where the simulated robot is (from the drive sim) */
    public VisionIOSim(Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose;
        AprilTagFieldLayout rebuilt = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        for (int i = 0; i < ids.length; i++) {
            Pose3d rebuiltPose = rebuilt.getTagPose(ids[i]).orElseThrow();
            Translation2d rebuiltHub = rebuiltPose.getX() > rebuilt.getFieldLength() / 2 ? REBUILT_RED_HUB : REBUILT_BLUE_HUB;
            Translation2d shift = PRACTICE_HUB.minus(rebuiltHub);
            tagPoses[i] = new Pose3d(
                rebuiltPose.getTranslation().plus(new Translation3d(shift.getX(), shift.getY(), 0.0)),
                rebuiltPose.getRotation());
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose3d camera = new Pose3d(robotPose.get()).transformBy(VisionSimConstants.SimRobotToCamera);

        int bestIndex = -1;
        double bestDistance = Double.MAX_VALUE, bestTx = 0, bestTy = 0;
        for (int i = 0; i < tagPoses.length; i++) {
            // The tag as the camera sees it: x forward, y left, z up.
            Pose3d inCamera = tagPoses[i].relativeTo(camera);
            double x = inCamera.getX(), y = inCamera.getY(), z = inCamera.getZ();
            if (x <= 0) {
                continue; // behind the camera
            }
            double tx = -Math.toDegrees(Math.atan2(y, x)); // Limelight: right is positive
            double ty = Math.toDegrees(Math.atan2(z, Math.hypot(x, y)));
            double distance = inCamera.getTranslation().getNorm();
            boolean inView = Math.abs(tx) <= VisionSimConstants.SimHorizontalFovDegrees / 2
                && Math.abs(ty) <= VisionSimConstants.SimVerticalFovDegrees / 2;
            if (!inView || distance > VisionSimConstants.SimMaxRangeMeters || !facesCamera(tagPoses[i], camera)) {
                continue;
            }
            if (distance < bestDistance) {
                bestIndex = i;
                bestDistance = distance;
                bestTx = tx;
                bestTy = ty;
            }
        }

        inputs.connected = true;
        inputs.latencyMs = 0.0;
        inputs.hasTarget = bestIndex >= 0;
        inputs.tagId = inputs.hasTarget ? ids[bestIndex] : -1;
        inputs.txDegrees = inputs.hasTarget ? bestTx : 0.0;
        inputs.tyDegrees = inputs.hasTarget ? bestTy : 0.0;
        inputs.distanceMeters = inputs.hasTarget ? bestDistance : 0.0;
        inputs.areaPercent = inputs.hasTarget ? areaPercent(bestDistance) : 0.0;

        // For AdvantageScope's 3D field: all hub tags, the camera, and the tag it sees.
        Logger.recordOutput("Vision/Sim/HubTags", tagPoses);
        Logger.recordOutput("Vision/Sim/CameraPose", camera);
        Logger.recordOutput("Vision/Sim/SeenTag", inputs.hasTarget ? new Pose3d[] {tagPoses[bestIndex]} : new Pose3d[0]);
    }

    /** A tag printed on the hub can only be read from the front, not from behind or edge-on. */
    private static boolean facesCamera(Pose3d tag, Pose3d camera) {
        Translation2d tagToCamera = camera.getTranslation().toTranslation2d().minus(tag.getTranslation().toTranslation2d());
        double angle = tagToCamera.getAngle().minus(tag.getRotation().toRotation2d()).getDegrees();
        return Math.abs(angle) < VisionSimConstants.SimMaxViewAngleDegrees;
    }

    /** Roughly how much of the image a tag fills at this distance (seen head-on). */
    private static double areaPercent(double distance) {
        double size = Math.toDegrees(2 * Math.atan(TAG_SIZE_METERS / 2 / distance));
        return 100.0 * (size / VisionSimConstants.SimHorizontalFovDegrees) * (size / VisionSimConstants.SimVerticalFovDegrees);
    }
}
