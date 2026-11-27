package frc.robot.subsystems.transfer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

public class Transfer extends SubsystemBase{
    private SparkMax transfer = new SparkMax(TransferConstants.transferMotor, MotorType.kBrushless);

    public enum WantedState {
        OFF,
        FORWARD,
        REVERSE
    }

    public enum CurrentState {
        OFF,
        FORWARD,
        REVERSE
    }
    public WantedState wantedState = WantedState.OFF;
    public CurrentState currentState = CurrentState.OFF;

    public CurrentState getTransferCurrentState() {
        return currentState;
}
    @Override
    public void periodic() {
        
        handleStates();
        applyStates();
}
}

