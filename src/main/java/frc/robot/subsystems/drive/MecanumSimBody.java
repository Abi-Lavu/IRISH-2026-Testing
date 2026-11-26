package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Constants.DriveConstants.DriveSimConstants;

/**
 * Simulation only: the robot as a solid physics body (maple-sim, which runs the dyn4j physics engine).
 *
 * Every physics step it pushes itself toward the speed the wheels are asking for, but never harder
 * than the wheels' grip allows. Because it's a real body with mass, it shoves balls out of the way
 * and stops when it runs into a wall or the hub.
 */
class MecanumSimBody extends AbstractDriveTrainSimulation {
    private static final double GRAVITY = 9.81;
    /** How quickly the robot catches up to the speed the wheels want (seconds). */
    private static final double RESPONSE_SECONDS = 0.05;

    /** Robot-relative speed the wheels are asking for. */
    private ChassisSpeeds target = new ChassisSpeeds();

    MecanumSimBody(Pose2d startPose) {
        super(
            DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(DriveSimConstants.SimRobotMassKg))
                .withBumperSize(
                    Meters.of(DriveSimConstants.SimBumperLengthMeters),
                    Meters.of(DriveSimConstants.SimBumperWidthMeters)),
            startPose);
    }

    void setTargetSpeeds(ChassisSpeeds robotRelative) {
        target = robotRelative;
    }

    @Override
    public void simulationSubTick() {
        ChassisSpeeds fieldTarget =
            ChassisSpeeds.fromRobotRelativeSpeeds(target, getSimulatedDriveTrainPose().getRotation());
        double mass = getMass().getMass();
        double maxForce = mass * GRAVITY * DriveSimConstants.SimWheelGrip;

        // Push toward the target speed (force = mass x acceleration), capped by wheel grip.
        Vector2 velocity = getLinearVelocity();
        double fx = mass * (fieldTarget.vxMetersPerSecond - velocity.x) / RESPONSE_SECONDS;
        double fy = mass * (fieldTarget.vyMetersPerSecond - velocity.y) / RESPONSE_SECONDS;
        double force = Math.hypot(fx, fy);
        if (force > maxForce) {
            fx *= maxForce / force;
            fy *= maxForce / force;
        }
        applyForce(new Vector2(fx, fy));

        // Same idea for turning.
        double maxTorque = maxForce * DriveSimConstants.SimTrackWidthMeters / 2;
        double torque = getMass().getInertia()
            * (fieldTarget.omegaRadiansPerSecond - getAngularVelocity()) / RESPONSE_SECONDS;
        applyTorque(Math.max(-maxTorque, Math.min(maxTorque, torque)));
    }
}
