package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import frc.robot.Constants.DriveConstants;


public class DriveIOTalonSRX implements DriveIO {
    private final TalonSRX[] motors = {
        new TalonSRX(DriveConstants.frontLeftMotorID),
        new TalonSRX(DriveConstants.frontRightMotorID),
        new TalonSRX(DriveConstants.backLeftMotorID),
        new TalonSRX(DriveConstants.backRightMotorID),
    };
    private final boolean[] inverted = {
        DriveConstants.frontLeftInverted,
        DriveConstants.frontRightInverted,
        DriveConstants.backLeftInverted,
        DriveConstants.backRightInverted,
    };
    private final Faults faults = new Faults();
    private final StickyFaults stickyFaults = new StickyFaults();

    public DriveIOTalonSRX() {
        for (int i = 0; i < motors.length; i++) {
            TalonSRX motor = motors[i];
            motor.configFactoryDefault();

            TalonSRXConfiguration config = new TalonSRXConfiguration();
            config.voltageCompSaturation = DriveConstants.VoltageCompSaturation;
            config.continuousCurrentLimit = DriveConstants.CurrentLimitAmps;
            config.peakCurrentLimit = 0;
            config.peakCurrentDuration = 0;
            config.openloopRamp = DriveConstants.OpenLoopRampSeconds;
            motor.configAllSettings(config);

            motor.setInverted(inverted[i]);
            motor.setNeutralMode(NeutralMode.Brake);
            motor.enableVoltageCompensation(true);
            motor.enableCurrentLimit(true);
        }
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        for (int i = 0; i < motors.length; i++) {
            TalonSRX motor = motors[i];
            inputs.busVolts[i] = motor.getBusVoltage();
            inputs.connected[i] = motor.getLastError() == ErrorCode.OK;
            inputs.appliedPercent[i] = motor.getMotorOutputPercent();
            inputs.statorAmps[i] = motor.getStatorCurrent();
            inputs.supplyAmps[i] = motor.getSupplyCurrent();
            inputs.tempC[i] = motor.getTemperature();
            motor.getFaults(faults);
            inputs.faults[i] = faults.toBitfield();
            motor.getStickyFaults(stickyFaults);
            inputs.stickyFaults[i] = stickyFaults.toBitfield();
        }
    }

    @Override
    public void setOutputs(double frontLeft, double frontRight, double backLeft, double backRight) {
        motors[0].set(TalonSRXControlMode.PercentOutput, frontLeft);
        motors[1].set(TalonSRXControlMode.PercentOutput, frontRight);
        motors[2].set(TalonSRXControlMode.PercentOutput, backLeft);
        motors[3].set(TalonSRXControlMode.PercentOutput, backRight);
    }
}
