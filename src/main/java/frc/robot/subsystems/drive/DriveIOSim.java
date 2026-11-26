package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveSimConstants;

/**
 * Fake drivetrain for simulation. There are no motors: each wheel's output is turned into a wheel
 * speed, the wheel speeds into robot movement, and that movement is added up into a pose so
 * AdvantageScope can draw the robot on the field.
 *
 * This is a rough model (no friction, no wheel slip). It's for seeing what the code does, not for
 * tuning real numbers.
 */
public class DriveIOSim implements DriveIO {
    private static final double LOOP_SECONDS = 0.02;

    // Where each wheel sits relative to the robot's center. Forward is +X, left is +Y.
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        new Translation2d(DriveSimConstants.SimWheelbaseMeters / 2, DriveSimConstants.SimTrackWidthMeters / 2), // FL
        new Translation2d(DriveSimConstants.SimWheelbaseMeters / 2, -DriveSimConstants.SimTrackWidthMeters / 2), // FR
        new Translation2d(-DriveSimConstants.SimWheelbaseMeters / 2, DriveSimConstants.SimTrackWidthMeters / 2), // BL
        new Translation2d(-DriveSimConstants.SimWheelbaseMeters / 2, -DriveSimConstants.SimTrackWidthMeters / 2)); // BR

    // The real Talons ramp from 0 to full in OpenLoopRampSeconds, so the fake ones do too.
    private final SlewRateLimiter[] ramps = new SlewRateLimiter[4];
    private final double[] targets = new double[4];
    private final double[] applied = new double[4];

    private Pose2d pose = DriveSimConstants.SimStartPose;

    public DriveIOSim() {
        double rate = 1.0 / Math.max(DriveConstants.OpenLoopRampSeconds, 0.01);
        for (int i = 0; i < 4; i++) {
            ramps[i] = new SlewRateLimiter(rate);
        }
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        for (int i = 0; i < 4; i++) {
            applied[i] = ramps[i].calculate(targets[i]);
            inputs.connected[i] = true;
            inputs.appliedPercent[i] = applied[i];
            inputs.busVolts[i] = 12.0;
            inputs.tempC[i] = 25.0;
        }

        // Percent output -> wheel speed in meters per second.
        double max = DriveSimConstants.SimMaxWheelSpeedMetersPerSec;
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
            applied[0] * max, applied[1] * max, applied[2] * max, applied[3] * max);

        // Wheel speeds -> how the whole robot moves, then add this loop's movement to the pose.
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(wheelSpeeds);
        pose = pose.exp(new Twist2d(
            speeds.vxMetersPerSecond * LOOP_SECONDS,
            speeds.vyMetersPerSecond * LOOP_SECONDS,
            speeds.omegaRadiansPerSecond * LOOP_SECONDS));
        inputs.simPose = pose;
    }

    @Override
    public void setOutputs(double frontLeft, double frontRight, double backLeft, double backRight) {
        targets[0] = frontLeft;
        targets[1] = frontRight;
        targets[2] = backLeft;
        targets[3] = backRight;
    }
}
