package frc.robot.subsystems.drive;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveSimConstants;


public class DriveIOSim implements DriveIO {
    
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        new Translation2d(DriveSimConstants.SimWheelbaseMeters / 2, DriveSimConstants.SimTrackWidthMeters / 2), // FL
        new Translation2d(DriveSimConstants.SimWheelbaseMeters / 2, -DriveSimConstants.SimTrackWidthMeters / 2), // FR
        new Translation2d(-DriveSimConstants.SimWheelbaseMeters / 2, DriveSimConstants.SimTrackWidthMeters / 2), // BL
        new Translation2d(-DriveSimConstants.SimWheelbaseMeters / 2, -DriveSimConstants.SimTrackWidthMeters / 2)); // BR

   
    private final SlewRateLimiter[] ramps = new SlewRateLimiter[4];
    private final double[] targets = new double[4];
    private final double[] applied = new double[4];

    private final MecanumSimBody body = new MecanumSimBody(DriveSimConstants.SimStartPose);

    
    public DriveIOSim() {
        double rate = 1.0 / Math.max(DriveConstants.OpenLoopRampSeconds, 0.01);
        for (int i = 0; i < 4; i++) {
            ramps[i] = new SlewRateLimiter(rate);
        }
        SimulatedArena.getInstance().addDriveTrainSimulation(body);
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

       
        double max = DriveSimConstants.SimMaxWheelSpeedMetersPerSec;
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(
            applied[0] * max, applied[1] * max, applied[2] * max, applied[3] * max));
        body.setTargetSpeeds(speeds);

        
        inputs.simPose = body.getSimulatedDriveTrainPose();
    }

    @Override
    public void setOutputs(double frontLeft, double frontRight, double backLeft, double backRight) {
        targets[0] = frontLeft;
        targets[1] = frontRight;
        targets[2] = backLeft;
        targets[3] = backRight;
    }
}
